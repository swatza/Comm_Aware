"""
Aircraft Communication Aware Application

Collects sensor measurements from a RF sensor (emulated or real), logs this information, calculates the error from a
prediction model and stores this. Uses this data to learn the error using a Gaussian Process and then predict
to create an estimated path loss field

Author: Spencer Watza
Copyright: CU Boulder RECUV

"""

# IMPORTS
import select
import time
import sys
import logging
import Queue
import argparse
import threading
import math
import struct
import socket
import serial
import numpy as np

sys.path.insert(0, '../PyUAS')  # get the path to the PyUAS Folder
sys.path.insert(0, '../PyUAS/protobuf')  # get the path to the protobuf format
import PyPacket
import PyPacketMsgBuilds
import PyPacketTypeCheck
import PyPackets_pb2
import Subscriber
import RF_Models
import assorted_lib
import PyPacketLogger
import Splat_Processing
import PyUAS_Ardupilot_Mission_Manager
import mav_command_wrappers

# rf_connect = "/dev/ttyUSB0" #how to read in rf signal data
rf_connect = "/dev/ttyS4"  # how to read in rf signal data -- beaglebone companion computer
# rf_connect = "/dev/bcn" #how to read in rf signal data -- Odroid companion computer (old)

# Shutdown event
shutdown_event = threading.Event()

# Outgoing message que
msg_queue = Queue.Queue()

# My Aircraft State for simulation
myState = 0

# Which nodes to plan on
planning_nodes = []
# Which Nodes to Learn
learning_nodes = []
# Which nodes to measure
measuring_nodes = []

previous_waypoint_pair= 0 #0 = A,B; 1 = B,C; 2 = A,C

def parseStateInfo(data, CLLA):
    # parse the state information
    msg = PyPackets_pb2.AircraftPixhawkState()
    msg.ParseFromString(data)

    # Should we just leave it as Lat/Lon for pn,pe?
    NED = assorted_lib.LLA2NED([msg.LLA_Pos.x, msg.LLA_Pos.y, msg.LLA_Pos.z], CLLA)
    # myState[0] #pn
    # myState[1] #pe
    # myState[3] #yaw
    # myState[5] #height
    # myState[6] #Airspeed
    # Set the state as 1A Kinematics for now
    global myState
    myState = [NED[0],NED[1],0,msg.attitude.z,0,NED[2],msg.airspeed]

def parseWaypointMsg(data, center):
    #parse into google buffer
    msg = PyPackets_pb2.Waypoint()
    msg.ParseFromString(data)
    loiter_radius = 50
    ttl = 300
    altitude = apmm.altitude # get the set altitude from the object

    # Get the relevant information
    if(msg.frame == "LLA"):
        gps_location = [msg.Pos.x, msg.Pos.y]
        new_command = mav_command_wrappers.createLoiterTimeCmd(ttl,loiter_radius,altitude,gps_location)
        apmm.goToGuided()
        apmm.resetMission()
        apmm.addToMission(new_command)
        apmm.activateMission()  # move the mission list into the command list buffer
        apmm.uploadCommands()  # upload that command list buffer to the ardupilot
        apmm.goToAuto()  # restart the mission (goes to mission 1)
    elif(msg.frame == "ENU"):
        enu = [msg.Pos.x,msg.Pos.y,msg.Pos.z]

        gps_location = assorted_lib.ENU2LLA(enu,center)
        new_command = mav_command_wrappers.createLoiterTimeCmd(ttl, loiter_radius, altitude, gps_location)
        apmm.goToGuided()
        apmm.resetMission()
        apmm.addToMission(new_command)
        apmm.activateMission()  # move the mission list into the command list buffer
        apmm.uploadCommands()  # upload that command list buffer to the ardupilot
        apmm.goToAuto()  # restart the mission (goes to mission 1)
    elif(msg.frame == "NED"):
        ned = [msg.Pos.x, msg.Pos.y, msg.Pos.z]
        gps_location = assorted_lib.NED2LLA(ned, center)
        new_command = mav_command_wrappers.createLoiterTimeCmd(ttl, loiter_radius, altitude, gps_location)
        apmm.goToGuided()
        apmm.resetMission()
        apmm.addToMission(new_command)
        apmm.activateMission()  # move the mission list into the command list buffer
        apmm.uploadCommands()  # upload that command list buffer to the ardupilot
        apmm.goToAuto()  # restart the mission (goes to mission 1)
    else:
        print("ERROR!!!: Unknown frame for new waypoint; switching to RTL")
        # mainlogger.error("Unknown Command Type: Switching to RTL ")
        apmm.goToRTL()

class TelemetryTask(threading.Thread):

    def __init__(self, my_id, sub_info_rf, sub_info_gcs, sub_info_state, port, ip, logmode, grid_info, clla):
        threading.Thread.__init__(self)

        # create logger
        self.logger = logging.getLogger("CommAwareApp:TelemetryTask")
        self.logger.setLevel(logmode)
        myhandler = logging.StreamHandler()
        self.logger.addHandler(myhandler)
        self.logger.info("CommAware Telemetry Task Started")

        # network setup
        self.PORT = port
        self.IP = ip
        self.ID = my_id
        self.NMPORT = 16000
        # netwrok output statement
        #d = {"port": self.PORT, "ip": self.IP, "id": self.ID, "Number": self.Num, "GCS": self.GCS_Num, "NMPort": self.NMPORT}
        #self.logger.debug("Initialized Values for the Telemetry: %s", ' ', extra=d)

        # Store the grid information
        self.region_grid_info = grid_info
        self.grid_x_points = np.arange(grid_info[0], grid_info[1] + 1, grid_info[2])
        self.grid_y_points = np.arange(grid_info[3], grid_info[4] + 1, grid_info[5])
        self.CLLA = clla

        # Generate a list of subscribers Here
        self.sublist = []
        # GCS COMMAND MSG
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_GCS_CMD,
                                                  PyPacket.PacketID(sub_info_gcs[0], sub_info_gcs[1]).getBytes(),
                                                  self.PORT, 'localhost', 1))  # GCS Command Msgs
        # RF STACKED MSG
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_WAYPOINT,
                                                  PyPacket.PacketID(sub_info_rf[0], sub_info_rf[1]).getBytes(),
                                                  self.PORT, 'localhost', 1))  # RF Model from GP Task
        # STATE MSG (Could be from a simulated source or not)
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_AUTOPILOT_PIXHAWK,
                                                  PyPacket.PacketID(sub_info_state[0], sub_info_state[1]).getBytes(),
                                                  self.PORT, 'localhost', 1))

        self.logger.debug("Finished initializing telemetry task")

    def run(self):
        global previous_waypoint_pair
        # Create Socket Objects
        my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        my_in_socket.bind(('', self.PORT))

        # Create lists for select
        in_sockets = [my_in_socket]
        out_sockets = [my_out_socket]

        # Add subscriber message to msg queue to be sent
        msg = PyPacketMsgBuilds.buildNodeHeartBeat(self.ID, self.sublist, 0)  # do we want to send this at set intervals?
        msg_queue.put(msg)
        lastsubtime = time.time()
        lastcmdtime = time.time()
        learn_counter = 1

        # Loop Forever
        while not shutdown_event.is_set():
            # Here is where we would put messages that send data at set intervals
            if (time.time() - lastsubtime) > 10:
                lastsubtime = time.time()
                # Should we update the packetnumber?
                msg_queue.put(msg)

            if(time.time() - lastcmdtime) > 60:
                lastcmdtime = time.time()
                # build command message
                pn = []
                print("Sending Learn Command")
                if planning_nodes[0] == "None":
                    wptflag = "False"
                    pn = planning_nodes
                elif len(planning_nodes) == 2:
                    wptflag = "True"
                    pn = planning_nodes
                    self.logger.info("Planning on the Error between %s, %s", pn[0], pn[1])
                elif len(planning_nodes) == 3:
                    if(previous_waypoint_pair == 0):
                        pn.append("NodeA")
                        pn.append("NodeB")
                    elif previous_waypoint_pair == 1:
                        pn.append("NodeB")
                        pn.append("NodeC")
                    elif previous_waypoint_pair == 2:
                        pn.append("NodeA")
                        pn.append("NodeC")
                    previous_waypoint_pair += 1
                    if previous_waypoint_pair > 2:
                        previous_waypoint_pair = 0
                    self.logger.info("Planning on the Error between %s, %s", pn[0], pn[1])
                #end elseif
                else:
                    pn = planning_nodes
                    wptflag = "False"
                cmd_msg = PyPacketMsgBuilds.buildRFLearnCommand(self.ID,learn_counter,learning_nodes,pn,wptflag,"error") #change this if we want to learn on the error or full data
                msg_queue.put(cmd_msg)

            # Check select
            readable, writable, exceptional = select.select(in_sockets, out_sockets, in_sockets)

            for s in readable:
                # get data from socket
                dataPkt, address = s.recvfrom(PyPacket.RECVBUFF)
                # assign datapkt as an object
                newPkt = PyPacket.PyPacket()
                newPkt.setPacket(dataPkt)
                # Parse packet
                if PyPacketTypeCheck.isWaypointMsg(newPkt):
                    self.logger.debug("Received Waypoint Msg: WIP")
                    #TODO! Handle the waypoint message
                    parseWaypointMsg(newPkt.getData(), self.CLLA)
                elif PyPacketTypeCheck.isAutoPilotPixhawkMsg(newPkt):  # used for simulating the aircraft's position
                    self.logger.debug("Received State Info Msg: Sim")
                    parseStateInfo(newPkt.getData(), self.CLLA)
                else:
                    # Unexpected message
                    self.logger.warning("Msg Parsing Problem: %s", 'Unexpected message type')
            # END

            #TODO! Update with while loop to make sure more messagse are sent instead of 1 at a time
            for s in writable:
                # Check to see if our output queue is empty
                while msg_queue.qsize() > 0:
                    try:
                        next_msg = msg_queue.get_nowait()
                    except Queue.Empty:
                        # queue is empty or blocked so stop looping
                        break
                    else:
                        s.sendto(next_msg, ('localhost', self.NMPORT))  # should always be localhost:NMport
                        self.logger.info("Message sent to: %s", ('localhost', self.NMPORT))
                    #END TRY
                #END WHILE QUE SIZE
        # END WHILE RUNNING

        my_out_socket.close()
        my_in_socket.close()

        self.logger.info("CommAware Telemetry Task [Closed]")


class SensingTask(threading.Thread):
    def __init__(self, antenna_names, altitude, grid_info, grid_info_2, logmode, center_pos, my_id, mode,clla):
        threading.Thread.__init__(self)

        # create logger
        self.logger = logging.getLogger("CommAwareApp:SensingTask")
        self.logger.setLevel(logmode)
        my_handler = logging.StreamHandler()
        self.logger.addHandler(my_handler)
        self.logger.info("CommAware Sensing Task Started")

        self.packet_log = PyPacketLogger.PyPacketLogger(
            ('Aircraft_' + my_id.getPlatform() + str(my_id.getIdentifier()) + '_Sensing_Task_Log'))
        self.packet_log.initFile()
        self.logger.info("Logging Sensor Packets to: %s", self.packet_log.logname)

        # important parameters
        self.center = center_pos
        self.refAlt = 0  # TODO! Why is this important?

        self.MYID = my_id  # handing it the object rather than the data

        # GridInfo = [-west, east, xspacing, -south, north, yspacing]
        # GridInfo2 = [x_grid_length,y_grid_length,x_center_index,y_center_index]
        self.xcenterIndex = grid_info_2[2]
        self.ycenterIndex = grid_info_2[3]
        self.xspacing = grid_info[2]
        self.yspacing = grid_info[5]
        self.xgridlength = grid_info_2[0]
        self.ygridlength = grid_info_2[1]

        # Load the propagation model into memory from file
        self.MyNodeMaps = {}
        self.MeasuredNodeMaps = {}
        self.AntennaGains = {}
        self.ReceiverGains = 3
        # add individual nodes to this map with the key being the name of the node and returns a dictionary
        for name in antenna_names:
            # find the file
            thisMap = {}
            secondMap = {}
            file = open(name + "_data_" + altitude + ".sdg", "r")
            #TODO! Is this right on the -1 part?
            for b in range(0, int(self.ygridlength)-1):
                for a in range(0, int(self.xgridlength)-1):
                    # set the byte number
                    byteNumber = Splat_Processing.findPLinFile(a, b, self.xgridlength-1)
                    self.logger.debug("seeking to file at %i", byteNumber)
                    file.seek(byteNumber)
                    bytesIn = file.read(4)
                    thisfloat = struct.unpack('f', bytesIn)
                    self.logger.debug("Retrieved float from file: %f at %i, %i", thisfloat[0], a, b)
                    thisMap[a, b] = thisfloat[0]
                    secondMap[a, b] = 0 #used for the measuredNodeMap
            # End for loop
            # Store into the main list of maps
            self.MyNodeMaps[name] = thisMap
            self.MeasuredNodeMaps[name] = secondMap
            # also need to store the AntennaGains at some point into the system
            # TODO! store antenna gains from file that we read in RoI?
            self.AntennaGains[name] = 3
        #         # End antenna file loop

        # If simulation
        self.SensorMode = mode  # 0 = simulation, 1 = real time, 2 = test
        self.RF_SimModel = RF_Models.Simple_RF_Model(1,CLLA=clla)  # load the simulation model here
        self.transmitter = {}
        self.transmitter["NodeA"] = RF_Models.RF_Transmitter(3,3,2400000000,[40.145217,-105.244637,self.refAlt])
        self.transmitter["NodeB"] = RF_Models.RF_Transmitter(3,3,2400000000,[40.120810,-105.244356,self.refAlt])
        # TODO! load the antenna file information from SPLAT!
        self.logger.info("Set RF Sim Model to Noise Model 1")

        # Serial link?

    def run(self):
        #Set all the global variables
        global myState
        global measuring_nodes
        # Build the data type
        rf_data_msg = PyPackets_pb2.RF_Data_Msg()
        rf_data_msg.ID = str(self.MYID.getBytes())

        # Build the packet
        newPacket = PyPacket.PyPacket()
        newPacket.setDataType(PyPacket.PacketDataType.PKT_RF_DATA_MSG)
        newPacket.setID(self.MYID.getBytes())

        # Sensor rate
        sensor_rate = 5  # Set to 5Hz
        rf_msg_counter = 0

        if self.SensorMode == 1:
            ser = serial.Serial(rf_connect)
            ser.reset_input_buffer()

        # Loop Forever
        time_last = time.time()
        while not shutdown_event.is_set():
            now_time = time.time()
            # sensor rate
            if (now_time - time_last) >= sensor_rate:
                time_last = time.time()
                self.logger.debug("Time to generate a measurement")
                rf_sensor_data = []  # initialize the list where i can store the multiple channels
                rf_sensor_chan = []
                # -----------------
                # Simulation Mode
                # -----------------
                if self.SensorMode == 0:
                    self.logger.debug("Sensor Mode 0")
                    # if our state is not null
                    if myState:
                        self.logger.debug("Found State: Proceeding to generate RF data")

                        # if the state is 1A Kinematics Model
                        # myState[0] #pn
                        # myState[1] #pe
                        # myState[3] #yaw
                        # myState[5] #height
                        # myState[6] #Airspeed

                        # Convert NED to ENU
                        LLA = assorted_lib.NED2LLA([myState[0], myState[1], myState[5]], self.center)

                        rf_data_msg.lla.x = LLA[0]
                        rf_data_msg.lla.y = LLA[1]
                        rf_data_msg.lla.z = LLA[2]

                        # optional Skip msg forms

                        # Determine my location in the grid
                        # -------X----------
                        xgrid = Splat_Processing.findGridIndex(myState[1], self.xspacing, self.xcenterIndex,
                                                               self.xgridlength-1)
                        # -------Y----------
                        ygrid = Splat_Processing.findGridIndex(myState[0], self.yspacing, self.ycenterIndex,
                                                               self.ygridlength-1)
                        self.logger.debug("Position is %f : %f", myState[1], myState[0])
                        self.logger.debug("Grids are %i : %i", xgrid, ygrid)
                        # RF Part
                        #TODO! Move this into a separate function for easy modification/debugging
                        for s in measuring_nodes:
                            node_name = s
                            self.logger.debug("Generating data for %s", node_name)
                            # Get the predicted path loss value
                            map = self.MyNodeMaps[node_name]
                            pl_predicted = map[xgrid, ygrid]
                            # create a new rf measurement for the message
                            new = rf_data_msg.rfNode.add()
                            new.chanID = node_name
                            #new.rssi = -150  # place holder
                            new.rssi = float(self.RF_SimModel.generateMeasurement(self.transmitter[s], [myState[1],myState[2],-myState[5]], self.ReceiverGains)) #Transmitter, ENU Position
                            # Calculate the measured path loss using the antenna gains (no antenna pointing assumed)
                            new.pl_msr = float(-(new.rssi - self.AntennaGains[node_name] - self.ReceiverGains))
                            # figure out what the prediction error is
                            new.pl_error = float(pl_predicted - new.pl_msr)
                            # add in the grid location
                            new.xgridNum = xgrid
                            new.ygridNum = ygrid
                        # end loop

                        # time
                        rf_data_msg.time = time.time()
                        rf_data_msg.packetNum = rf_msg_counter
                        # increment counter
                        rf_msg_counter = rf_msg_counter + 1

                        # Log
                        self.logger.info("RF Data Msg number %i with time value: %f", rf_data_msg.packetNum,
                                         rf_data_msg.time)

                        # serialize
                        data_str = rf_data_msg.SerializeToString()

                        # Put the data into a packet
                        newPacket.setData(data_str)
                        # log packet
                        self.packet_log.writePacketToLog(newPacket)

                        # Add the packet to the global send queue
                        msg_queue.put(newPacket.getPacket())
                        del rf_data_msg.rfNode[:]

                    else:
                        self.logger.warning('No state found')
                        time.sleep(1)

                # -----------------
                # Normal Mode
                # -----------------
                elif self.SensorMode == 1:
                    # Pull vehicle data using dronekit
                    rf_data_msg.lla.x = vehicle.location.global_frame.lat
                    rf_data_msg.lla.y = vehicle.location.global_frame.lon
                    rf_data_msg.lla.z = vehicle.location.global_relative_frame.alt
                    # Optional
                    rf_data_msg.attitude.x = vehicle.attitude.roll
                    rf_data_msg.attitude.y = vehicle.attitude.pitch
                    rf_data_msg.attitude.z = vehicle.attitude.yaw
                    rf_data_msg.airspeed = vehicle.airspeed

                    # -First determine indexes
                    ENU = assorted_lib.LLA2ENU([rf_data_msg.lla.x, rf_data_msg.lla.y, rf_data_msg.lla.z],
                                               [vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, self.refAlt])
                    # -------X----------
                    xgrid = Splat_Processing.findGridIndex(ENU[0], self.xspacing, self.xcenterIndex, self.xgridlength)
                    # -------Y----------
                    ygrid = Splat_Processing.findGridIndex(ENU[1], self.yspacing, self.ycenterIndex, self.ygridlength)

                    # RF data part
                    #TODO! Pull out into a separate function for easy modification
                    try:
                        line = ser.readline()
                        entries = line.split(",")
                        counter = 0
                        for s in range(0, len(entries)):
                            if entries[s].startswith('C'):
                                if counter == len(measuring_nodes):
                                    break
                                else:
                                    # ==============================
                                    # ASSUMING CH1 == NodeA, etc....
                                    # ==============================
                                    # THIS MIGHT NEED TO BE FIXED; validate with hardware
                                    rf_chan = entries[s]
                                    rf_data = entries[s + 1], entries[s + 2]  # TODO! which is omni which is directional?
                                    # Believe these are not needed
                                    # rf_sensor_chan.append(rf_chan)
                                    # rf_sensor_data.append(rf_data)

                                    self.logger.info("Got measurements from sensor")
                                    node_name = measuring_nodes[counter]
                                    map = self.MyNodeMaps[node_name]
                                    pl_predicted = map[xgrid, ygrid]

                                    new = rf_data_msg.rfNode.add()
                                    #CHannel ID is actually which node its coming from
                                    new.chanID = node_name
                                    # calculate the "measured path loss"
                                    # TODO! Update with new method of getting channel gains
                                    msred_pl = -(float(rf_data[0]) - self.AntennaGains[node_name] - self.ReceiverGains)
                                    # the error between measured and predicted
                                    pl_prediction_error = pl_predicted - msred_pl

                                    new.rssi = float(rf_data[0])
                                    new.rssi2 = float(rf_data[1])
                                    new.pl_msr = float(msred_pl)
                                    new.pl_error = float(pl_prediction_error)
                                    new.xgridNum = xgrid
                                    new.ygridNum = ygrid
                                    # increment counter for stuffing into node stuff
                                    counter += 1
                                    self.logger.info("Survived a loop of getting sensor measurements")
                            else:
                                pass

                        # time
                        rf_data_msg.time = time.time()
                        rf_data_msg.packetNum = rf_msg_counter
                        # increment counter
                        rf_msg_counter = rf_msg_counter + 1

                        # Log
                        self.logger.info("RF Data Msg number %i with time value: %f", rf_data_msg.packetNum,
                                         rf_data_msg.time)

                        # serialize
                        data_str = rf_data_msg.SerializeToString()

                        # Put the data into a packet
                        newPacket.setData(data_str)
                        # log packet
                        self.packet_log.writePacketToLog(newPacket)

                        # Add the packet to the global send queue
                        msg_queue.put(newPacket.getPacket())
                        del rf_data_msg.rfNode[:]

                    except:
                        # log message
                        self.logger.warning("serial error with sensors")
                    # End try loop


                # -----------------
                # Test Mode
                # -----------------
                elif self.SensorMode == 2:
                    # Test outputs with just hardcoded numbers for now
                    rf_data_msg.packetNum = 999
                    rf_data_msg.time = time.time()
                    rf_data_msg.lla.x = 40.130692
                    rf_data_msg.lla.y = -105.244599
                    rf_data_msg.lla.z = 1700  # ASL ?
                    rf_data_msg.attitude.x = 0
                    rf_data_msg.attitude.y = 0
                    rf_data_msg.attitude.z = 0
                    rf_data_msg.airspeed = 15

                    for ind in range(0, 3):
                        newnode = rf_data_msg.rfNode.add()
                        if ind == 0:
                            newnode.chanID = "NodeA"
                            newnode.rssi = -62
                            newnode.pl_msr = 97
                            newnode.pl_error = -12
                        elif ind == 1:
                            newnode.chanID = "NodeB"
                            newnode.rssi = -48
                            newnode.pl_msr = 80
                            newnode.pl_error = -4
                        elif ind == 2:
                            newnode.chanID = "NodeC"
                            newnode.rssi = -55
                            newnode.pl_msr = 88
                            newnode.pl_error = 9

                        newnode.xgridNum = 5
                        newnode.ygridNum = 10

                    # Log
                    self.logger.info("RF Data Msg number %i with time value: %f", rf_data_msg.packetNum,
                                     rf_data_msg.time)

                    # serialize
                    data_str = rf_data_msg.SerializeToString()

                    # Put the data into a packet
                    newPacket.setData(data_str)
                    # log packet
                    self.packet_log.writePacketToLog(newPacket)

                    # Add the packet to the global send queue
                    msg_queue.put(newPacket.getPacket())
                    del rf_data_msg.rfNode[:]


                # -----------------
                # Unknown Mode
                # -----------------
                else:
                    self.logger.critical("Unknown Mode Detected: No data is being generated. Quitting!")
                    sys.exit()
                # END MODE LOOPS
            else:
                time.sleep(0.01)
            # END FREQ LOOPS
        # END WHILE

        self.logger.info("Closing Sensor Task")
        print('\t SensorTask [closed]')

if __name__ == "__main__":
    # Load a simulation file that has starting location and simulation model info
    mainlogger = logging.getLogger("CommAwareApp:Main")

    # Parse Arguments; For now assume defaults
    # TODO! Fix the argument system; more complex and simpler to use
    parser = argparse.ArgumentParser(description='Comm-Aware Autonomous Application for aircraft')
    parser.add_argument('SYSTEM_PLATFORM', type=str)
    parser.add_argument('SYSTEM_NUMBER', type=int)
    parser.add_argument('GROUND_CONTROL_PLATFORM', type=str)
    parser.add_argument('GROUND_CONTROL_NUMBER', type=int)
    parser.add_argument('LEARNING_PLATFORM', type=str)
    parser.add_argument('LEARNING_NUMBER', type=int)
    parser.add_argument('ROI_FILE', type=str)
    parser.add_argument('ALTITUDE', type=str)
    parser.add_argument('MEASURING_NODES', type=str)  # list like NodeA,NodeB
    parser.add_argument('LEARNING_NODES',type=str)
    parser.add_argument('PLANNING_NODES',type=str)
    parser.add_argument('LOGMODE',type=int)
    parser.add_argument('MODE',type=int)
    # Nodes to plan on

    # Parse the argument information
    args = parser.parse_args()
    # Handle the information to correct variables
    RoI_FILE = args.ROI_FILE
    ALTITUDE = args.ALTITUDE
    IP = 'localhost'
    PORT = 14300

    planning_nodes = []
    if args.PLANNING_NODES == "Triangle":
        planning_nodes.append("NodeA")
        planning_nodes.append("NodeB")
        planning_nodes.append("NodeC")
    elif args.PLANNING_NODES == "Simple":
        planning_nodes.append("NodeA")
        planning_nodes.append("NodeB")
    elif args.PLANNING_NODES == "None":
        print "Not Planning on this mission"
        planning_nodes = args.PLANNING_NODES
    else:
        print "Critical Error: Unknown Planning Scheme"

    learning_nodes = args.MEASURING_NODES.split(",")
    measuring_nodes = args.LEARNING_NODES.split(",")

    # Parse this programs ID information (Platform and Number)
    SYSTEM_PLATFORM = PyPacket.PlatformDispatch[PyPacket.formatPlatformString(args.SYSTEM_PLATFORM)]()
    SYSTEM_NUMBER = args.SYSTEM_NUMBER
    MY_ID = PyPacket.PacketID(SYSTEM_PLATFORM, SYSTEM_NUMBER)

    TARGET_PLATFORM = PyPacket.PlatformDispatch[PyPacket.formatPlatformString(args.LEARNING_PLATFORM)]()
    TARGET_NUMBER = args.LEARNING_NUMBER
    SUB_INFO_RF = [TARGET_PLATFORM, TARGET_NUMBER]

    TARGET_PLATFORM = PyPacket.PlatformDispatch[PyPacket.formatPlatformString(args.GROUND_CONTROL_PLATFORM)]()
    TARGET_NUMBER = args.GROUND_CONTROL_NUMBER
    SUB_INFO_GCS = [TARGET_PLATFORM, TARGET_NUMBER]

    SUB_INFO_STATE = [PyPacket.PacketPlatform.AIRCRAFT, SYSTEM_NUMBER]

    # HARD coded debug mode for now
    LOGMODE = args.LOGMODE

    # Load Meta Data from Region File
    meta_data = Splat_Processing.loadMetaData(RoI_FILE)
    # mainlogger.critical("Couldn't find and open RoI File")

    # parse the meta data, everything else will be grabbed when needed
    # Calculate the X and Y Grid lengths
    x_grid_length = (meta_data[4] + meta_data[5]) / meta_data[6] + 1
    y_grid_length = (meta_data[2] + meta_data[3]) / meta_data[7] + 1
    # center index
    x_center_Index = math.ceil(x_grid_length / 2)  #TODO! I think the center position is off by 1
    y_center_Index = math.ceil(y_grid_length / 2)

    # GridInfo = [-west, east, xspacing, -south, north, yspacing]
    Grid_Info = [-meta_data[5], meta_data[4], meta_data[6], -meta_data[3], meta_data[2], meta_data[7]]
    # GridInfo2 = [x_grid_length,y_grid_length,x_center_index,y_center_index]
    Grid_Info_2 = [x_grid_length, y_grid_length, x_center_Index, y_center_Index]
    # centerPoint = [lat,lon]
    center_Point = [meta_data[0], meta_data[1], 0]
    # Pull out antenna information
    Antenna = meta_data[8]
    # Pull out the antenna names
    Antenna_Names = []
    strvalues = []
    for ant in Antenna:
        strvalues = ant.split(",")
        Antenna_Names.append(strvalues[0])
        # TODO! Anything else to retrieve from this bits of information?

    # Mode 0,1,2 (Sim, Operational, Test)
    MODE_FLAG = args.MODE

    if MODE_FLAG == 1:

        connection_string = '/dev/ttyS1' # BeagleBone Setup
        # connection_string = 'udp:0.0.0.0:14551'
        print 'Connecting to vehicle on: %s' % (connection_string)
        apmm = PyUAS_Ardupilot_Mission_Manager.ArduPilotMissionManager(connection_string, "aircraft_test", ALTITUDE, [meta_data[0],meta_data[1]])
        # vehicle = connect(connection_string, baud=57600, _initialize=True, wait_ready=None)
        vehicle = apmm.vehicle

    # =================================
    # Start Telemetry Thread
    telem = TelemetryTask(MY_ID, SUB_INFO_RF, SUB_INFO_GCS, SUB_INFO_STATE, PORT, IP, LOGMODE, Grid_Info, center_Point)
    telem.start()
    # Start Sensing Thread
    sensing = SensingTask(Antenna_Names, ALTITUDE, Grid_Info, Grid_Info_2, LOGMODE, center_Point, MY_ID, MODE_FLAG, center_Point)
    sensing.start()
    # =================================

    while threading.active_count() > 2:
        try:
            time.sleep(1)
            # Here is where we will do the local determination of which waypoints to follow or something.

        except(KeyboardInterrupt, SystemExit):
            # log message
            shutdown_event.set()
        # end try
    # end while
    # log ending message
    sys.exit()
