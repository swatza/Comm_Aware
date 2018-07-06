#!/bin/bash

#Start the aircraft comm aware app
#ARGS:
#System Platform
#System Number
#Ground Control Platform
#Ground Control Number
#Learning Platform
#Learning Number
#RoI File
#Altitude
#measuring Nodes
#learning nodes
#planning nodes 
#logmode
#mode 
python aircraft_comm_aware_app.py aircraft 50 groundstation 1 aircraft 50 RoI_Antennas.txt 100 NodeA,NodeB,NodeC NodeA,NodeB None 10 0

