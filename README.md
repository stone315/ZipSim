# ZipSim

## Run code
To execute the code, it is required to install python3 and requirements.txt.

```bash
pip install -r requirements.txt
```

Use the following command to run the game.
```bash
python zip_sim.py
```

To enable autopilot (my_pilot.py), run the following command.
```bash
python zip_sim.py python my_pilot.py [--options--]

--options--

"-a" or "--agent" : [ choose the type of agent.
                      available agents: Native, Reflex, Test
                      default agnet: Native]
                      
"--trees" : [ show the detected trees ]

"--sites" : [ show the detected sites ]
```

You can also run multiple times of simultions at once.
```bash
./run.sh [number of simulations] [agent]

ie.
./run.sh 1000 Reflex
```

## Summary
In the project, the goal os to develop differenet automonous agents to meet the requirements of ZIPAA, ZAA, and more under different environmental condition.

- Double-delivery rate less than 1 in 1000 flights (ZIPAA)
- Crash rate less than 1 in 100 flights (ZAA)
- Parachute rate less than 1 in 10 flights
- Deliver at least 10% of packages on average

### Result
Both Native agent and Reflex agent can meet 3 out of 4 requirements (ZIPAA, Parachute rate < 10%, and delivery rate > 10%). In fact, both agents have 0% double-delivery rate and 0% parachute rate. Also, they outperform the requirement of delivery rate, usually between 70% - 100%. For the Crash rate, Native agent has appoximate 93.2% survival rate while Reflex agent has 99.2% survival rate. Therefore, only Reflex agent meets the ZAA requirement.

## Methodology
The problem can be broken down to four main subproblems:

1. Path Planning (Search all objects)
2. Perception (Distinguish between Tree and Delivert Site)
3. Path Planning (avoid the obstacles)
4. Decision ( Determine the timing to deploy package)

Native agent and Reflex agent use the same methods to solve probem 1.,2.,4. However, they use different methods to avoid the obstacles. The Native agent relies on pre-calcualtion result that it plans the path using the map gererated from the perceoption. Instead, Reflex agent uses the real time data to find out the safe path which may cost more energy for the extre movement. 

### Assumption
Native agent and Reflex agent rely on a few assumptions.

- The shapes of Trees and Sites are consistant ( 0.5 meter and 3 meters radius representatively)
- Trees and Sites are not closed to each other
- Wind velocity is considered as dummy variable which can be compensated by applying opposite velocity. No prediction of the wind.
- The measurement is precise. There is no measurement error or noise.

### Path Planning (Search all objects)
It is less important in this project because of the powerful sensors. The sensors can measure up to 255 meters object. It can cover 132 meters in width that excesses the need (50 meters). Therefore, I apply a simple scheme for this problem; The drone stay at 0.0 (y-aixs) if no unknown object is detected. If there is unknown object, it moves between -7 and 7 to identify it. The scheme is implemented in **getAction** function.

![alt text](https://github.com/stone315/ZipSim/blob/main/pic/problem1.png)

### Perception (Distinguish between Tree and Delivert Site)
The program performs one of the easest way to identify objects with different sizes. In different distance, the measurement accuracy between two points is different. For example, in 10m away, the distance between two measured point is 10m * 0.01745 = 0.1745m. Therefore, the sensors can identify an object as small as 0.1745m long. If an object has 6m diameter, 34.38 sampling points can be recorded. On the other side, a site (1m diameter) has only 5.73 sampling points. Therefore, we can identify the object based on the number of sampling points recorded in different relative distance. The implementation is in **identigyObject** function

Conditions:

To identify a tree, the number of sampling points is needed to be greater than the maximum possible number of site's sampling point.

To identigy a site, the number of sampling points is needed to be less than the minimum number of tree's sampling point and it is not in the tree list.



![alt text](https://github.com/stone315/ZipSim/blob/main/pic/measurement.png)

| distance      | accuracy | tree (6m diameter) |  site (1m diameter) |
| ------------- | ------------- | ------------- | ------------- | 
| r | 0.01745r  | - | - |
| 100 m  | 1.745m  | 3.45 sampling points | 0.5714 sampling points |
| 50 m  | 0.872m | 6.88 sampling points | 1.1467 sampling points |
| 25 m  | 0.436m | 13.76 sampling points | 2.25 sampling points |
| 10 m  | 0.1745m | 34.38 sampling points | 5.73 sampling points |



#### Center of site
Since the radius of site is very small (0.5m) compared to the site's radius (5m), the coordinate of the sampling point is near to the center. Let the point be the center does not affect the result.

#### Center of tree
The center of tree can be found by solving a set of mathematical equations. First, the points p1 and p2 can be computed using the sensors' data and the formula ![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn6.svg) and ![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn7.svg). 

Then, p3 is the mid point between p1 and p2. The coordinate of p3 can be found using mid point theorem. 

![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn3.svg)

The points p1 and p2 are at the edge of circle. Also, the slope of p1-p2 is perpendicular to the slope of center-p3. Solving the two equations can find the center of tree.

![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn5.svg)

![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn4.svg)


![alt text](https://github.com/stone315/ZipSim/blob/main/pic/center.png)

### Decision ( Determine the timing to deploy package)

The timing of dropping packages should hold this equation. The package should land between d + r and d - r. If the package requires ts time to travel to d, then the velocity should be d/ts. It can also be expressed as

![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn.svg)

![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn2.svg)

- d is the distance between the delivery site and the drone.
- r is the radius of the delivery site (5m)
- v is the velocity
- x is the velocity in x-aixs (sum of lateral velocity and wind velocity)
- y is the velocity in x-aixs (sum of forward velocity and wind velocity)
- ts is the package's falling time ( I choose 4s for better result)

Although the drone may not pass the center, the equation works in most of the cases. The function **boolDeploy** can check the validity of the equation. If the equation holds, it drops a package.

To guarantee ZIPAA, the function **boolDeploy** also check wheather a package is dropped in last 10 m.

![alt text](https://github.com/stone315/ZipSim/blob/main/pic/problem%204.png)

### Path Planning (Avoid the obstacles)

#### Native Agent
 
The Native Agent decides the path based on information of the trees' locations, and the delivery site's locations. The locations are detected by the **identifyobject** function and stored in self.unknown, self.trees and self.delivery_site list. The motion action is determined in the **getAction** function.

First, the **getAction** function sorts the self.unknown and self.trees and chooses the closest object (comparing the vertical distances) as the target position. Then, check whether the current position is in dangerous zone or not. There are three different cases.

1. It is in safe zone. Then, find the safe position that is closed to target, like the right figure below.
2. It is near the dangerous zone but the front sensor (sample[15]) do not detect obstacle. Then, move forward only.
3. It is in dangerous zone. Then, find the closed safe position and move toward the position, like the left figure below.
 
This native agent cannot avoid some of the obstacles. The primary reason is that the computed center of trees has error. The error can be up to 3m that can affect the correctness of plan. 


![alt text](https://github.com/stone315/ZipSim/blob/main/pic/safeZone.png)
 
#### Reflex Agent
The Reflex Agent has much better performance than Native Agent in respect to the crash prevention. It is due to the relfex agent relying on the current sensing data to make the safest action.

The **getAction** function will read 0th to 30th sensors' data. If the measured distance is less than 30m, then it is possible to hit in next 1s, so it is not safe. After finding out the safe angles, the **getAction** function will choose the a lowest turning angle as target angle.

![alt text](https://github.com/stone315/ZipSim/blob/main/pic/reflex.png)

The tangent of turning angle is equal to the ratio of (lateral speed + wind speed in y-axis) to forward speed.

![equation](https://github.com/stone315/ZipSim/blob/main/pic/CodeCogsEqn8.svg)

![alt text](https://github.com/stone315/ZipSim/blob/main/pic/problem3.png)

## More Idea
Native agent and Reflex agent are very simple agent. Both rely on many assumptions that are impossible in the real world. This project is very fun and there are a lot of space can improve. In here, I try to list some of the idea and possible solutions.

1. Better Path Planning

   Alougth the program works, a lot of movement is redundant. The extra movement can cause more energy/gas and also miss the delivery sites. 
   
   **Possible solution**: graphical search algorithm (Dijkstra's algorithm, A*, or Uniform search)

2. Learning agent

   When there is not enough environment information, learning agent may perform better, such as no information about the shape of tree and the size of delivery site.
   
   **Possible solution**: Reinforment learning, Mokev Model
   
3. measurement is not accurate and noisy + dynamic object

   In the real world, the sensor's data is more noisy that cannot perfectly measure the distance. Especially, when an object is moving, the distance can be vary. Fault is also possible to happen.
   
   **Possible solution**: Kalman filtering, Particle filter

4. Predict the wind velocity

   The program treat the wind velocity as a dummy variable. However, practically, the wind can push a drone to an unwanted direct and destory the drone. Estimation of wind velocity can provide information to controller so that the controller can counter the wind velocity. 
   
   **Possible solution**: design a optimal controller and observer

