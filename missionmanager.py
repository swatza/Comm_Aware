from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
import math
from pymavlink import mavutil
import numpy as np

class MissionManager(object):
    def __init__(self, connection, n_control, n_plan, altitude, home, name):
        # connection 	- dronekit vehicle connection
        # n_control 	- control horizon (int: # of waypoints)
        # n_plan 		- planning horizon (int: # of waypoints)
        # altitude 		- altitude (Above Ground Level) of the waypoints in meters
        # home 			- aircraft home in (lat,lon)
        # name 			- aircraft name

        print "Connecting to vehicle on: %s" % connection
        self.vehicle = connect(connection, wait_ready=True)
        self.n_control = n_control
        self.n_plan = n_plan
        self.altitude = altitude
        self.last_path = np.empty([0,0])
        self.slot = 2
        self.wp_jump = (self.slot - 1)*(self.n_plan + 2) + 1
        self.commands = self.vehicle.commands
        newcmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, home[0], home[1], self.altitude)
        self.commands.add(newcmd)
        self.commands.clear()
        self.name = name

    def printCommandList(self):
        # Prints a CommandList object to the terminal
        print "Mission in %s's Command List:" % self.name
        next_pt = self.vehicle.commands.next
        # Print the command list
        for cmd in self.commands:
            lat=cmd.x
            lon=cmd.y
            alt=cmd.z
            if i == (next_pt-1): #TODO! WHAT IS GOING ON HERE
                print '#', cmd.seq, '-', cmd.command, ":", " Lat: ", lat, " Lon: ", lon, " Alt: ", alt, ' <<<'
            elif cmd.command == 177:
                print '#', cmd.seq, '-', cmd.command, ": Jump to waypoint # ", cmd.param1
            elif cmd.command == 20:
                print '#', cmd.seq, '-', cmd.command, ": RTL"
            else:
                print '#', cmd.seq, '-', cmd.command, ":", " Lat: ", lat, " Lon: ", lon, " Alt: ", alt


    def createMissionList(self, pts, id_jump, wp_jump):
        # Converts the lat, lon info in pts and the altitude constant into a mission list of autopilot commands.
        # A loiter and RTL are added to the end of the planned path as a safety precaution

        missionlist = [] # initialize the mission list
        missionlist = [] # initialize the mission list
        path_length = pts.shape[1] # The number of columns in the path list
        for i in range(0, path_length + 2):
            if i == id_jump:
                # Creates a jump command to the jump waypoint
                newcmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 0, wp_jump, -1, 0, 0, 0, 0, 0)
            elif i < path_length:
                # Creates a waypoint command with the given path list
                newcmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pts[0,i], pts[1,i], self.altitude)
            elif i == path_length:
                # Creates a loiter command at the last point in the path list
                newcmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS, 0, 0, 3, 0, 0, 0, pts[0,i-1], pts[1,i-1], self.altitude)
            else:
                # Creates an RTL command after the loiter command
                newcmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, self.altitude)
            missionlist.append(newcmd) # add the new command object to the missionlist
        return missionlist

    def setCommandList(self, missionlist):
        # Creates a CommandList object with commands in missionlist
        cmds = self.commands
        cmds.clear()
        for cmd in missionlist:
            cmds.add(cmd)
        return cmds

    def switchSlots(self):
        # Index bookkeeping for the ring buffer
        self.slot = (self.slot) % 2 + 1 # switches the slot number between 1 and 2 and vice versa
        self.wp_jump = (self.slot - 1)*(self.n_plan + 2) + 1

    def setVehicleMission(self, newpts):
        # Used by KAIST to upload a 2 by n_plan numpy array, newpts, to the aircraft.
        # [lat_1, lat_2, ... , lat_n;
        #  lon_1, lon_2, ... , lon_n]
        newmission = self.createMissionList(newpts, -1, -1)
        if self.last_path.size == 0: # this is the first time uploading a mission to the aircraft
            print "Initializing vehicle path."
            print "Resetting ring buffer size with appropriately sized command list...."
            self.setCommandList(newmission)
            print "Done."
        else: # stitch in new trajectory with the old mission
            print "Stitching plan into current mission...."
            oldmission_jump = self.createMissionList(self.last_path, self.n_control, self.wp_jump)
            if self.slot == 1:
                missionlist = newmission + oldmission_jump
            elif self.slot == 2:
                missionlist = oldmission_jump + newmission
            self.setCommandList(missionlist)
            self.switchSlots()
            print "Done."

        # Upload mission to A/C
        print "Uploading Commands to aircraft...."
        self.commands.upload()
        print "Done."

        # Save the last uploaded path
        self.last_path = newpts

    def ready(self):
        # Outputs True if the aircraft is ready to receive the next plan
        next_wp = self.vehicle.commands.next
        start = (self.slot % 2)*(self.n_plan + 2) + 1
        end = start + self.n_control - 1
        return (start <= next_wp <= end) or (self.last_path.size == 0)

    def missedJump(self):
        next_wp = self.vehicle.commands.next
        if ( self.n_control + 2 <= next_wp < self.n_plan + 2 ):
            print "%s missed jump command.  Changing next point to waypoint %s" % (self.name, self.n_plan + 3)
            self.vehicle.commands.next = self.n_plan + 3
            print "Next mission point is now %s." % self.vehicle.commands.next
            return True
        elif ( self.n_plan + self.n_control + 4 <= next_wp ):
            print "%s missed jump command.  Changing next point to waypoint %s" % (self.name, 1)
            self.vehicle.commands.next = 1
            print "Next mission point is now %s." % self.vehicle.commands.next
            return True
        else:
            return False

    def uploadTakeoff(self):
        missionlist = []
        cmd_TO = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, self.altitude)
        cmd_RTL = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, self.altitude)
        missionlist.append(cmd_TO)
        missionlist.append(cmd_RTL)
        self.setCommandList(missionlist)
        self.commands.upload()

    def vehicleState(self):
        print "Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame
        print "Attitude: %s" % self.vehicle.attitude
        print "Velocity: %s" % self.vehicle.velocity
        print "Groundspeed: %s" % self.vehicle.groundspeed
        print "Last Heartbeat: %s" % self.vehicle.last_heartbeat

