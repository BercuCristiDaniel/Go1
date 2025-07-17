#!/usr/bin/env python3
import rospy
import math
import actionlib
import tf
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from scipy.ndimage import distance_transform_edt
from heapq import heappop, heappush
import numpy as np

from sklearn.cluster import DBSCAN
from scipy.ndimage import binary_dilation

class PathFindingAlgorithm:
    def __init__(self):
        # Initialisation des abonnements et publications
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_robot_position)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.update_goal_position)

        self.global_plan_pub = rospy.Publisher("/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.actual_path_pub = rospy.Publisher("/actual_path", Path, queue_size=10)
        self.reached_points_pub = rospy.Publisher("/reached_points", PoseStamped, queue_size=10)

        self.safety_margin = 0.45  # Marge de sécurité en mètres
        self.safety_table_margin = 0.40  # Marge de sécurité en mètres

        self.step = 10

        # Liste pour enregistrer les poses de la trajectoire
        self.actual_trajectory = []
        self.actual_trajectory = []
        self.reached_points = []

        # Données de la carte, position du robot et position de l'objectif
        self.map_data = None
        self.robot_position = None
        self.goal_position = None
        self.initial_position = None

        # Variables de trajet
        self.planned_trajectory = []  # Stocker la trajectoire sinusoïdale
        self.current_target_index = 0  # Indice pour suivre le point actuel

        # Action client pour envoyer des objectifs à move_base
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()



    def map_callback(self, msg):
        rospy.loginfo(f"Carte de {msg.info.width}x{msg.info.height} reçue avec une résolution de {msg.info.resolution} mètre/cellule.")
        self.map_data = msg

    def update_robot_position(self, msg):
        """Mise à jour de la position actuelle du robot."""
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Position actuelle du robot : {self.robot_position}")

        # Enregistrer la pose actuelle dans la trajectoire réelle parcourue
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.robot_position[0]
        pose.pose.position.y = self.robot_position[1]
        pose.pose.orientation.w = 1.0
        self.actual_trajectory.append(pose)
        self.publish_actual_path()
        rospy.loginfo(f"État du client d'action après modification de la : {self.move_base_client.get_state()}")
        rospy.loginfo(f"Fin d'une update de la position : {self.robot_position}")

    def calculate_orientation(self, from_point, to_point):
        """
        Calcule l'orientation en quaternion entre deux points.
        :param from_point: Tuple (x, y) du point de départ
        :param to_point: Tuple (x, y) du point d'arrivée
        :return: Quaternion (x, y, z, w)
        """
        delta_x = to_point[0] - from_point[0]
        delta_y = to_point[1] - from_point[1]
        yaw = math.atan2(delta_y, delta_x)  # Angle en radians
        
        rospy.loginfo(f"Calculating orientation: from {from_point} to {to_point}, yaw={yaw}")
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        return quaternion


    def update_goal_position(self, msg):
        """Mise à jour de la position de l'objectif et génération de la trajectoire sinusoïdale."""
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo(f"Nouvel objectif défini : {self.goal_position}")

            # Annulez tout objectif en cours
        current_state = self.move_base_client.get_state()
        if current_state in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            rospy.logwarn("Annulation de l'objectif en cours avant de définir un nouvel objectif.")
            self.move_base_client.cancel_goal()
            rospy.sleep(0.5)  # Pause pour permettre au client de se stabiliser

        # Réinitialiser l'index du point courant
        self.current_target_index = 0
        rospy.sleep(1)  # Attendre que move_base se stabilise
        
        if self.robot_position:
            # Générer et publier la trajectoire sinusoïdale
            #self.publish_curved_path()
            # Générer et publier la trajectoire A*
            self.publish_astar_path()
            
            # Commencer à suivre la trajectoire
            self.send_next_goal()
            rospy.loginfo(f"First point of the trajectory are follow")

    def publish_curved_path(self):
        """Trace une courbe entre la position du robot et l'objectif."""
        if not self.robot_position or not self.goal_position:
            rospy.logwarn("Position du robot ou de l'objectif non définie, impossible de générer le chemin.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # Réinitialiser la trajectoire existante
        self.planned_trajectory = []

        # Génération des poses avec une courbe sinusoïdale entre le point de départ et le point d'arrivée
        num_points = 5
        x_start, y_start = self.robot_position
        x_end, y_end = self.goal_position

        for i in range(0, num_points + 1):
            t = i / num_points
            x = (1 - t) * x_start + t * x_end
            y = (1 - t) * y_start + t * y_end + 1.5 * math.sin(2 * math.pi * t)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

            self.planned_trajectory.append(pose)

        if len(self.planned_trajectory) < 2:
            rospy.logerr("La trajectoire générée est trop courte. Vérifiez les paramètres.")
            return

        # Publie la trajectoire sinusoïdale
        self.path_pub.publish(path_msg)
        rospy.loginfo("Trajectoire sinusoïdale publiée sur /planned_path")

        # Publie également sur le topic du planificateur global
        # self.global_plan_pub.publish(path_msg)
        rospy.loginfo("Trajectoire sinusoïdale publiée sur le topic du planificateur global /move_base/TrajectoryPlannerROS/global_plan")

    def publish_astar_path(self):
        """Génère un chemin avec A* et le publie."""
        if not self.robot_position or not self.goal_position:
            rospy.logwarn("Position du robot ou de l'objectif non définie.")
            return

        grid = self.generate_grid()
        if grid is None:
            return

        start_cell = (
            int((self.robot_position[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution),
            int((self.robot_position[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution),
        )
        goal_cell = (
            int((self.goal_position[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution),
            int((self.goal_position[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution),
        )


        # Validation des cellules de départ et d'arrivée
        if grid[start_cell[0], start_cell[1]] == 1:
            rospy.logwarn(f"La cellule de départ {start_cell} est dans la marge de sécurité ou sur un obstacle. Recherche de la cellule libre la plus proche...")
            start_cell = self.find_nearest_free_cell(grid, start_cell) or start_cell
        else:
            rospy.loginfo(f"La cellule de départ {start_cell} est valide.")

        if grid[goal_cell[0], goal_cell[1]] == 1:
            rospy.logwarn(f"La cellule d'arrivée {goal_cell} est dans la marge de sécurité ou sur un obstacle. Recherche de la cellule libre la plus proche...")
            goal_cell = self.find_nearest_free_cell(grid, goal_cell) or goal_cell
        else:
            rospy.loginfo(f"La cellule d'arrivée {goal_cell} est valide.")



        if not (0 <= start_cell[0] < grid.shape[0] and 0 <= start_cell[1] < grid.shape[1]):
            rospy.logwarn("Position de départ hors limites de la carte.")
            return
        if not (0 <= goal_cell[0] < grid.shape[0] and 0 <= goal_cell[1] < grid.shape[1]):
            rospy.logwarn("Position de l'objectif hors limites de la carte.")
            return


        rospy.loginfo(f"Calcul du chemin A* de {start_cell} à {goal_cell}.")

          # Réduction progressive de la marge
        margin_step = 0.1  # Réduction de la marge à chaque tentative
        min_margin = 0.1  # Marge minimale
        path_cells = None

        #path_cells = self.a_star(start_cell, goal_cell, grid)

        # Si le chemin n'est pas trouvé, réduire la marge de sécurité
        #path_cells = self.a_star(start_cell, goal_cell, grid)
        
        while path_cells is None:
            path_cells = self.a_star(start_cell, goal_cell, grid)

            if not path_cells:
                # Réduire la marge si aucun chemin trouvé
                rospy.logwarn("Echec du calcul A*. Réduction de la marge de sécurité...")
                #grid = self.recalculate_with_smaller_margin(grid, self.safety_margin, margin_step, min_margin)
                self.a_star(start_cell, goal_cell, grid)

                if grid is None:  # Si aucune solution n'est trouvée même avec une marge réduite
                    rospy.logerr("Aucun chemin valide trouvé, même avec des marges réduites.")
                    return

        if not path_cells:
            rospy.logwarn("Aucun chemin trouvé par A*.")
            return

        # Conversion du chemin en Path ROS
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        # Réinitialiser la trajectoire existante
        self.planned_trajectory = []

            # Ajouter un segment pour relier le départ réel à la première cellule libre
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.pose.position.x = self.robot_position[0]
        start_pose.pose.position.y = self.robot_position[1]
        start_pose.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose)

        first_path_pose = PoseStamped()
        first_path_pose.header.frame_id = "map"
        first_path_pose.pose.position.x = start_cell[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        first_path_pose.pose.position.y = start_cell[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        first_path_pose.pose.orientation.w = 1.0
        path_msg.poses.append(first_path_pose)

        for i, cell in enumerate(path_cells):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = cell[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
            pose.pose.position.y = cell[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

            # Calculer l'orientation pour tous sauf le dernier point
            if i < len(path_cells) - 1:
                next_cell = path_cells[i + 1]
                next_point = (
                    next_cell[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x,
                    next_cell[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y,
                )
                current_point = (
                    pose.pose.position.x,
                    pose.pose.position.y,
                )
                quaternion = self.calculate_orientation(current_point, next_point)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
            else:
                # Orientation par défaut pour le dernier point
                pose.pose.orientation.w = 1.0

            rospy.loginfo(f"Ajout d'un point avec orientation : Position ({pose.pose.position.x}, {pose.pose.position.y}), "
                        f"Orientation (x={pose.pose.orientation.x}, y={pose.pose.orientation.y}, "
                        f"z={pose.pose.orientation.z}, w={pose.pose.orientation.w})")
            
             # Vérification avant d'ajouter
            if pose in self.planned_trajectory:
                rospy.logwarn(f"Point déjà présent dans planned_trajectory : ({pose.pose.position.x}, {pose.pose.position.y})")

            path_msg.poses.append(pose)
            self.planned_trajectory.append(pose)

        # Ajouter un segment pour relier la dernière cellule libre à la position réelle de l'arrivée
        last_path_pose = PoseStamped()
        last_path_pose.header.frame_id = "map"
        last_path_pose.pose.position.x = goal_cell[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        last_path_pose.pose.position.y = goal_cell[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        last_path_pose.pose.orientation.w = 1.0
        path_msg.poses.append(last_path_pose)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = self.goal_position[0]
        goal_pose.pose.position.y = self.goal_position[1]
        goal_pose.pose.orientation.w = 1.0
        path_msg.poses.append(goal_pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Chemin A* publié sur /planned_path.")

    def send_next_goal(self):
        """Envoie le prochain point de la sinusoïde comme objectif."""
        rospy.loginfo(f"Go to the point {self.current_target_index} of the trajectory")

        if self.current_target_index < len(self.planned_trajectory):

            target_pose = self.planned_trajectory[self.current_target_index]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            #goal.target_pose.pose = target_pose.pose
            goal.target_pose.pose.position.x = target_pose.pose.position.x
            goal.target_pose.pose.position.y = target_pose.pose.position.y
            goal.target_pose.pose.position.z = target_pose.pose.position.z
            
            # Assignez l'orientation calculée au MoveBaseGoal
            goal.target_pose.pose.orientation.x = target_pose.pose.orientation.x
            goal.target_pose.pose.orientation.y = target_pose.pose.orientation.y
            goal.target_pose.pose.orientation.z = target_pose.pose.orientation.z
            goal.target_pose.pose.orientation.w = target_pose.pose.orientation.w

            # Log pour vérifier l'orientation
            rospy.loginfo(f"Envoi de l'objectif : Position ({goal.target_pose.pose.position.x}, {goal.target_pose.pose.position.y}), "
                      f"Orientation (x={goal.target_pose.pose.orientation.x}, y={goal.target_pose.pose.orientation.y}, "
                      f"z={goal.target_pose.pose.orientation.z}, w={goal.target_pose.pose.orientation.w})")

            # Annulez l'objectif actuel pour éviter tout conflit
            rospy.loginfo("Annulation de l'objectif actuel (s'il existe).")
            self.move_base_client.cancel_goal()
            self.move_base_client.wait_for_result(timeout=rospy.Duration(2.0))

            # Réinitialisez le client d'action
            rospy.loginfo("Réinitialisation du client d'action.")
            self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            self.move_base_client.wait_for_server()

            # Envoyez le nouvel objectif
            rospy.loginfo(f"Envoi du point {self.current_target_index} comme nouvel objectif.")
            self.move_base_client.send_goal(goal, done_cb=self.goal_reached_callback)

            rospy.sleep(0.1)  # Pause courte pour s'assurer que l'objectif est actif
            new_state = self.move_base_client.get_state()
            rospy.loginfo(f"État après envoi : {new_state}")

            

        else:
            rospy.loginfo("Tous les points de la trajectoire ont été atteints.")

        # # Passer au prochain point en sautant les autres
        if len(self.planned_trajectory)-self.current_target_index<self.step:
            self.current_target_index = len(self.planned_trajectory)
        else:
            self.current_target_index += self.step
            

    def goal_reached_callback(self, status, result):
        """Callback pour gérer l'atteinte de chaque objectif."""
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Point {self.current_target_index} Reach.")

             # Vérifiez si l'index courant est valide dans la trajectoire planifiée
        if self.current_target_index < len(self.planned_trajectory):
            # Obtenir le point correspondant
            reached_point = self.planned_trajectory[self.current_target_index]

        #     # Publier le point atteint sur le topic "/reached_points"
            reached_point = self.planned_trajectory[self.current_target_index]
            self.reached_points.append(reached_point)

            self.reached_points_pub.publish(reached_point)

            # Réinitialisation pour éviter tout conflit
            rospy.loginfo("Réinitialisation après succès.")
            self.move_base_client.cancel_goal()
            rospy.sleep(0.5)  # Délai pour stabiliser l'état

            if self.current_target_index < len(self.planned_trajectory):
                rospy.sleep(0.5)  # Délai pour la transition
                self.send_next_goal()
                rospy.loginfo(f"Passage au point suivant : {self.current_target_index}")
                rospy.logwarn(f"le nombre de point est de:{len(self.planned_trajectory)}")
            else:
                rospy.loginfo("Tous les points de la trajectoire ont été atteints.")
        elif status in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED]:
            rospy.logwarn("Objectif annulé. Tentative d'envoi du prochain objectif.")
            self.send_next_goal()
        else:
            rospy.logwarn("État inattendu. Réessayer l'objectif actuel.")
            self.send_next_goal()

    def enhance_table_safety(self, grid, additional_margin):
        """
        Détecte les petits amas (comme des pieds de table) et augmente la marge de sécurité autour d'eux.
        :param grid: Grille numpy où 0 = libre, 1 = obstacle.
        :param additional_margin: Marge supplémentaire en mètres autour des petits amas.
        :return: Grille modifiée avec des marges de sécurité augmentées autour des petits amas.
        """
        # Trouver les indices des obstacles
        obstacle_indices = np.argwhere(grid == 1)

        if len(obstacle_indices) == 0:
            rospy.logwarn("Aucun obstacle détecté dans la carte.")
            return grid

        # Utiliser DBSCAN pour détecter les clusters d'obstacles
        eps_in_cells = int(0.1 / self.map_data.info.resolution)  # Exemple : 10 cm converti en cellules
        clustering = DBSCAN(eps=eps_in_cells, min_samples=3).fit(obstacle_indices)  # Ajuster eps et min_samples
        labels = clustering.labels_

        # Création d'une copie de la grille pour ajouter les marges
        enhanced_grid = grid.copy()

        for cluster_id in set(labels):
            
            if cluster_id == -1:
                continue  # Ignorer les "bruits" (points non assignés)
            
            rospy.logwarn("Cluster détecter")
            # Extraire les indices des cellules appartenant à ce cluster
            cluster_indices = obstacle_indices[labels == cluster_id]

            # Vérifier si c'est un petit amas (nombre limité de points)
            if len(cluster_indices) < 800:  # Ajuster ce seuil selon la taille typique des pieds de table
                    # Dilater ce cluster pour ajouter une marge de sécurité
                cluster_mask = np.zeros_like(grid, dtype=bool)
                cluster_mask[tuple(zip(*cluster_indices))] = True

                # Convertir la marge supplémentaire en cellules
                margin_in_cells = int(additional_margin)
                dilated_cluster = binary_dilation(cluster_mask, iterations=margin_in_cells)

                # Ajouter le cluster dilaté à la grille améliorée
                enhanced_grid[dilated_cluster] = 1

                rospy.logwarn("Cluster détecter et petit")

        return enhanced_grid

    def generate_grid(self):
        if not self.map_data:
            rospy.logwarn("Données de carte non disponibles.")
            return None

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution

        grid = np.array(self.map_data.data).reshape((height, width))
        grid = np.where(grid > 50, 1, 0)  # 1 = obstacle, 0 = libre

        # Convertir la carte en une grille numpy
        grid = np.array(self.map_data.data).reshape((height, width))
        grid = np.where(grid > 50, 1, 0)  # 1 = obstacle, 0 = libre

        # Calculer la marge de sécurité en nombre de cellules
        margin_in_cells = int(self.safety_margin / resolution)
        table_margin_in_cell = int(self.safety_table_margin / resolution)

        # Dilater les obstacles pour ajouter la marge de sécurité
        dilated_grid = self.dilate_obstacles(grid, margin_in_cells)

        # Ajouter une marge de sécurité spécifique pour les petits amas
        enhanced_grid = self.enhance_table_safety(dilated_grid, additional_margin=table_margin_in_cell)

        self.publish_grid(enhanced_grid)
        
        rospy.loginfo(f"Grille générée avec une marge de sécurité de {self.safety_margin} m ({margin_in_cells} cellules).")
    
        return enhanced_grid
        #return grid

    def a_star(self, start, goal, grid):
        """
        Implémente l'algorithme A* pour trouver un chemin optimal sur une grille.
        :param start: Tuple (x, y) position de départ
        :param goal: Tuple (x, y) position de destination
        :param grid: Grille numpy où 0 = libre et 1 = obstacle
        :return: Liste des positions (x, y) formant le chemin
        """
        def heuristic(a, b):
            # Heuristique Euclidienne pour mieux gérer les diagonales
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        rows, cols = grid.shape
        #open_set = []self.dilate_obstacles
        open_set = []  # Initialisation de la liste des nœuds à explorer
        heappush(open_set, (0, start))  # Ajouter le point de départ avec une priorité de 0
        came_from = {}  # Dictionnaire pour garder la trace du chemin
        g_score = {start: 0}  # Coût du chemin du départ à chaque nœud
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Chemin de départ à destination

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # 8 directions
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Vérification des limites de la grille
                if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                    continue

                # Vérification des obstacles
                if grid[neighbor[0], neighbor[1]] == 1:
                    continue

                # Vérification des collisions en diagonale
                if abs(dx) == 1 and abs(dy) == 1:  # Déplacement diagonal
                    if grid[current[0] + dx, current[1]] == 1 or grid[current[0], current[1] + dy] == 1:
                        continue

                tentative_g_score = g_score[current] + heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        rospy.logwarn("Aucun chemin trouvé par A*.")
        return []

    def dilate_obstacles(self, grid, margin, resolution=1.0):
        """
        Élargit les obstacles dans la grille pour ajouter une marge de sécurité.
        :param grid: Grille numpy où 0 = libre et 1 = obstacle
        :param margin: Marge de sécurité en mètres
        :param resolution: Résolution de la carte en mètres par cellule
        :return: Grille dilatée
        """
        if not isinstance(margin, (int, float)) or margin <= 0:
            raise ValueError("La marge de sécurité (margin) doit être un nombre positif.")
        
        # Convertir la marge en cellules
        margin_in_cells = int(margin / resolution)

        #rospy.loginfo(f"Dilatation avec marge {current_margin:.2f} m ({margin_in_cells} cellules).")
        
        # Calculer la distance de chaque cellule au plus proche obstacle
        distance = distance_transform_edt(grid == 0)

        # Débogage (facultatif)
        print(f"distance shape: {distance.shape}, dtype: {distance.dtype}")
        print(f"margin_in_cells: {margin_in_cells}, max distance: {np.max(distance)}")
        
        # Marquer les cellules à moins de `margin_in_cells` comme obstacles
        dilated_grid = (distance <= margin_in_cells).astype(int)

        return dilated_grid


    def find_nearest_free_cell(self, grid, cell, max_radius= 20):
        """
        Trouve la cellule libre la plus proche d'une cellule donnée.
        :param grid: Grille numpy où 0 = libre et 1 = obstacle
        :param cell: Tuple (x, y) de la cellule à valider
        :param max_radius: Rayon maximal en cellules pour chercher une cellule libre
        :return: Coordonnées de la cellule libre la plus proche ou None si introuvable
        """
        rows, cols = grid.shape
        x, y = cell

        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0:
                        return (nx, ny)

        rospy.logwarn(f"Impossible de trouver une cellule libre proche de {cell}.")
        return None

    def publish_actual_path(self):
        """Publie le chemin réel parcouru pour visualisation dans Rviz."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.actual_trajectory
        self.actual_path_pub.publish(path_msg)
        rospy.loginfo("Chemin réel publié sur /actual_path")

    def publish_grid(self, grid, frame_id="map", topic_name="/visualized_grid"):
        """
        Publie une grille (numpy array) en tant qu'OccupancyGrid pour la visualiser dans Rviz.
        :param grid: Grille numpy (0 = libre, 1 = obstacle)
        :param frame_id: Frame de référence
        :param topic_name: Nom du topic pour publier la grille
        """
        grid_pub = rospy.Publisher(topic_name, OccupancyGrid, queue_size=10)
        
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = frame_id

        occupancy_grid.info.resolution = self.map_data.info.resolution
        occupancy_grid.info.width = grid.shape[1]
        occupancy_grid.info.height = grid.shape[0]
        occupancy_grid.info.origin = self.map_data.info.origin

        # Convertir la grille numpy en une liste 1D pour OccupancyGrid
        occupancy_grid.data = (grid.flatten() * 100).astype(np.int8).tolist()

        grid_pub.publish(occupancy_grid)
        rospy.loginfo(f"Grille publiée sur {topic_name}.")


    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("path_finding_algorithm")
    rospy.sleep(1)  # Pause pour s'assurer de la connexion
    rospy.loginfo("Démarrage de PathFindingAlgorithm.")
    pfa = PathFindingAlgorithm()
    pfa.run()