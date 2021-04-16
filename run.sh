#!/bin/bash

var=0
run=$1

echo "Number of simulation: $run"
 
for i in $(seq 1 $run);
do
  python zip_sim.py --headless python my_pilot.py -a Reflex

  if [ $? -eq 0 ]
  then
    ((var = var + 1))
  fi
done

echo "result is $var"
