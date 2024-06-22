![Untitled design (2)](https://github.com/devanys/PRM-mobile-robot/assets/145944367/4db39160-c5dc-4843-8296-f861937839b6)

### Implementation of Probabilistic Roadmap (PRM)

Probabilistic Roadmap (PRM) is a method used in trajectory planning for mobile robots in complex and dynamic environments. This approach combines the concept of environment mapping with random sampling techniques to build a graphical representation of the robot configuration space. The main goal of PRM is to create a network of connected points (nodes) with safe trajectories (edges) and enable the robot to move from the starting position to the destination efficiently. involves the following steps:

1. **Environmental Mapping**:
 - **Data Processing**: Data from sensors is processed to build a digital representation of the environment around the robot.

2. **Probabilistic Roadmap Formation**:
 - **Nodes**: PRM uses sample points randomly placed in the robot's configuration space, representing the start position, destination, and intermediate points.
 - **Road Connections**: For each sample point, the algorithm determines the path connections between points that can be passed without colliding with obstacles.

3. **Optimal Path Search**:
 - **Search Algorithm**: Algorithms such as A* or Dijkstra are used to search for the optimal path between the start and destination points in a graph consisting of sample points as vertices and road connections as edges.
![image](https://github.com/devanys/PRM-mobile-robot/assets/145944367/cbcb24ea-b4d2-42c4-acb3-3b54052e2b04)
![image](https://github.com/devanys/PRM-mobile-robot/assets/145944367/b28f5f82-0ae5-4134-8002-ceaaa912ac6d)

4. **Implementation on Mobile Robot**:
 - **Robot Control**: The resulting trajectory is executed by the robot control system to move the robot from the starting point to the destination, taking into account the dynamics of the robot and other obstacles.
 - **Trajectory Update**: The robot can update its trajectory planning if any changes in the environment are detected.

PRM implementation enables mobile robots to perform trajectory planning efficiently in complex and changing environments. By utilizing random samples and search algorithms, PRM can address navigation challenges by considering the accuracy, speed, and safety of robot movement.

### Contributors
- Dr. Muhammad Fuad, S.Kom., M.T
- M Fat Hiy Ilman N
- Astri Nur P
- Siti Nurhalisa
- Devan Yusfa S
- Firman Aliyansyah S
### Special Thanks
- ### **Dr. Muhammad Fuad, S.Kom., M.T**
We greatly appreciated for providing the foundational theories and serving as the instructor for the Mobile Robot course. Expertise and guidance have been invaluable to our learning experience.

