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
3. Path Planning (Prevent the obstacles)
4. Decision ( Determine the timing to deploy package)

Native agent and Reflex agent use the same methods to solve probem 1.,2.,4. However, they use different methods to prevent the obstacles. The Native agent relies on pre-calcualtion result that it plans the path using the map gererated from the perceoption. Instead, Reflex agent uses the real time data to find out the safe path which may cost more energy for the extre movement. 

### Assumption
Native agent and Reflex agent rely on a few assumptions.

- The shapes of Trees and Sites are consistant ( 0.5 meter and 3 meters radius representatively)
- Trees and Sites are not closed to each other
- Wind velocity is considered as dummy variable which can be compensated by applying opposite velocity. No prediction of the wind.
- The measurement is precise. There is no measurement error or noise.

### Path Planning (Search all objects)
It is less important in this project because of the powerful sensors. The sensors can measure up to 255 meters object. It can cover 132 meters in width that excesses the need (50 meters). Therefore, I apply a simple scheme for this problem; The drone stay at 0.0 (y-aixs) if no unknown object is detected. If there is unknown object, it moves between -7 and 7 to identify it. The scheme is implemented in **getAction** function.

![alt text](https://github.com/stone315/ZipSim/blob/main/problem1.png)

### Perception (Distinguish between Tree and Delivert Site)

### Decision ( Determine the timing to deploy package)
The timing of dropping packages should hold this equation.



![alt text](https://github.com/stone315/ZipSim/blob/main/problem%204.png)
### Path Planning (Prevent the obstacles)
