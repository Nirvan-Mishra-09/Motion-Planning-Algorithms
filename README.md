# dijkstra_path_finding
Implementation of Dijkstra Algorithm in the given map.

................................................................................................

Libraries used:

import cv2

import numpy as np


import heapq

................................................................................................

Code Overview:

Step 1: Using opencv to create a map, with clearance given by the user.

![map](https://github.com/Nirvan-Mishra-09/Motion-Planning-Algorithms/assets/127642231/1da78fa1-47a0-4468-833a-e8b9965f92b1)




Step 2: Creating user interface and checking for valid inputs.

Step 3: To define action set of the point robot, this robot can move in 8 directions up, down, left, right, up_right, up_left, down_right, and down_left. 

Cost of up, down, left, and right moves are '1', on the other hand the remaining moves have '1.4' as their costs.

![actionset](https://github.com/Nirvan-Mishra-09/dijkstra_path_finding/assets/127642231/8e3ed3e3-67d6-48b5-8b4b-be4f995d1931)


Step 4: Implementing dijkstra's algorithm, and finding the shortest path from start point and destination. 

![Untitled Diagram](https://github.com/Nirvan-Mishra-09/dijkstra_path_finding/assets/127642231/c1a20639-5480-41ed-a294-42c8a243c181)


Step 5: Results 

User input: 

![results](https://github.com/Nirvan-Mishra-09/dijkstra_path_finding/assets/127642231/14f8c7b7-574e-4fa2-b641-088cb1a9ea84)


Path taken:





https://github.com/Nirvan-Mishra-09/Motion-Planning-Algorithms/assets/127642231/d4177d9d-343c-48b2-996b-fb68dcc3ac8d







