# Import DroneKit-Python
import os
import time
import string
import sys
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
import math
import numpy as np
from pymavlink import mavutil
from missionmanager import MissionManager
import GetWaypoints as wp
import multiprocessing

def printVehicleMission(vehicle, cmds):
	print "Mission on Aircraft:"

	next_pt = vehicle.commands.next

	# Save the vehicle commands to a list
	i = 0
	for cmd in cmds:
		lat=cmd.x
		lon=cmd.y
		alt=cmd.z
		targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
		if i == (next_pt-1):
			print '#', cmd.seq, '-', cmd.command, ":", " Lat: ", lat, " Lon: ", lon, " Alt: ", alt, ' <<<'
		elif cmd.command == 177:
			print '#', cmd.seq, '-', cmd.command, ": Jump to waypoint", cmd.param1
		elif cmd.command == 20:
			print '#', cmd.seq, '-', cmd.command, ": RTL"
		else:
			print '#', cmd.seq, '-', cmd.command, ":", " Lat: ", lat, " Lon: ", lon, " Alt: ", alt
		i = i+1

if __name__ == "__main__":
	clear = lambda : os.system('clear')

	#PARAMETERS:
	connection_string = "udp:0.0.0.0:14550"
	aSize = 50						#meters to demo box
	aircraftWaypointAlt = 100		#meters
	pauseLoiterTurns = 3			#Will loiter this many times before RTL
	wp_radius=10					#meters; desired wp_radius (default 10)
	dt = 10.0						# time (s) defining space between waypoints
	V = 20							# aircraft velocity (m/s)
	baseDT = 0.05					# the dt used to generate each full dummy path (paths saved in text file to be read in)
	id_strt = 1						# initial waypoint point where we would like next path to start from
	n_plan = 12						# number of waypoints in each pan
	n_control = 8					# horizon size (number of waypoints to be flown in each plan)
	home = 40.138000,-105.240699	# starting location (lla) of mission
	pathtxtfile = 'Path21.txt'		# text file that contains full dummy path
	next_pt = 0						# next waypoint in the autopilot
	waypoint_delay = 5.0 			# time delay for the waypoint function
	in_sim = True					# Simulation flag (MAKE FALSE IF USING HARDWARE!)

	home_wp = wp.Location_lla(40.1451,-105.245)

	# Waypoint generation setup
	print "Loading preset flight path from file: %s..." % pathtxtfile
	path = np.asarray(wp.choosepath(pathtxtfile,dt,baseDT))
	print "Done."

	# Initialize MissionManager Object
	Talon_mission = MissionManager(connection_string, n_control, n_plan, aircraftWaypointAlt, home, 'Talon')

	if in_sim and Talon_mission.vehicle.mode.name != "RTL":
		Talon_mission.uploadTakeoff()
		print "Ready for takeoff."
		while Talon_mission.vehicle.mode.name != "RTL":
			#wai
			time.sleep(0.5)
		print "Mode switch to RTL confirmed."
		print "Ready for mode switch to AUTO."

	while True:
		if (next_pt != Talon_mission.vehicle.commands.next):
			# clear()
			printVehicleMission(Talon_mission.vehicle, Talon_mission.commands)
			next_pt = Talon_mission.vehicle.commands.next
			print "Current vehicle mode is: %s" % Talon_mission.vehicle.mode.name

		if Talon_mission.ready():
			# Calculate lat, lon waypoints for the next trajectory
			# KAIST Placeholder
			traj_LLA, id_strt = wp.waypoints(path, n_plan, n_control, id_strt,dt, V, home_wp)
			time.sleep(waypoint_delay)

			# Upload pts
			Talon_mission.setVehicleMission(traj_LLA)

		# Check if jump waypoint was missed
		Talon_mission.missedJump()

		# Wait
		time.sleep(0.1)