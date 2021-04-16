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
Both Native and Reflex agent can meet 3 out of 4 requirements (ZIPAA, Parachute rate < 10%, and delivery rate > 10%). In fact, both agents have 0% double-delivery rate and 0% parachute rate. Also, they outperform the requirement of delivery rate, usually between 70% - 100%. For the Crash rate, Native agent has appoximate 93.2% survival rate while Reflex agent has 99.2% survival rate. Therefore, only Reflex agent meets the ZAA requirement.
