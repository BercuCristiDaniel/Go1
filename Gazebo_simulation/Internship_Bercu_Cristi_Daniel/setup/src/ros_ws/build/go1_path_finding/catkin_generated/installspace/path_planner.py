#!/usr/bin/env python3

import rospy
import heapq
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        # Subscriptions
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Publications
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)

        # Stored data
        self.map_data = None
        self.start = None
        self.goal = None

    def map_callback(self, msg):
        self.map_data = msg
        rospy.loginfo("Map received.")

    def start_callback(self, msg):
        self.start = (
            int((msg.pose.pose.position.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution),
            int((msg.pose.pose.position.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        )
        rospy.loginfo(f"Start position set to: {self.start}")

    def goal_callback(self, msg):
        self.goal = (
            int((msg.pose.position.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution),
            int((msg.pose.position.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        )
        rospy.loginfo(f"Goal position set to: {self.goal}")
        if self.map_data and self.start and self.goal:
            self.plan_path()

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_neighbors(self, pos, grid):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        neighbors = []
        for dx, dy in directions:
            neighbor = (pos[0] + dx, pos[1] + dy)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor] == 0:  # Free space
                    neighbors.append(neighbor)
        return neighbors

    def a_star(self, start, goal, grid):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.get_neighbors(current, grid):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        rospy.logwarn("No path found.")
        return None

    def plan_path(self):
        rospy.loginfo("Planning path using A*...")
        resolution = self.map_data.info.resolution
        grid = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        grid = np.where(grid > 50, 1, 0)

        if grid[self.start] == 1 or grid[self.goal] == 1:
            rospy.logwarn("Start or goal position is blocked.")
            return

        path_cells = self.a_star(self.start, self.goal, grid)

        if path_cells:
            rospy.loginfo("Path found!")
            path_msg = Path()
            path_msg.header.frame_id = "map"

            for i, cell in enumerate(path_cells):
                pose = PoseStamped()
                pose.pose.position.x = cell[1] * resolution + self.map_data.info.origin.position.x
                pose.pose.position.y = cell[0] * resolution + self.map_data.info.origin.position.y

                # Calculate yaw dynamically if there's a next waypoint
                if i < len(path_cells) - 1:
                    next_cell = path_cells[i + 1]
                    dx = next_cell[1] - cell[1]
                    dy = next_cell[0] - cell[0]
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0  # Default yaw for the last waypoint

                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
        else:
            rospy.logwarn("No path found!")

if __name__ == '__main__':
    try:
        PathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
