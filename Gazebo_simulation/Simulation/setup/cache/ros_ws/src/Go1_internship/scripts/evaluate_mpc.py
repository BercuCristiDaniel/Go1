import time
import numpy as np

if __name__ == '__main__':
    i = 1
    average_vector = np.array([])

    while i <100:
        start = time.time()
        i=i+1
        end = time.time()
        average_vector = np.append(average_vector,end - start)

    print(np.mean(average_vector))