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
