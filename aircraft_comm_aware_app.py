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
import PyPackets_pb2
import Subscriber
import RF_Models
import assorted_lib
import PyPacketLogger
import Splat_Processing

# rf_connect = "/dev/ttyUSB0" #how to read in rf signal data
rf_connect = "/dev/ttyS4"  # how to read in rf signal data -- beaglebone companion computer
# rf_connect = "/dev/bcn" #how to read in rf signal data -- Odroid companion computer (old)

# Shutdown event
shutdown_event = threading.Event()

# Outgoing message que
msg_queue = Queue.Queue()

# My Aircraft State for simulation
myState = 0

# Which Nodes to Learn
planning_nodes = []
# Which nodes to measure
measuring_nodes = []


# Global Pixhawk Status information

def isCommand(pkt):
    if pkt.getDataType() == PyPacket.PacketDataType.PKT_GCS_CMD:
        # log received gcs cmd message
        return True
    else:
        return False


def isLearnedModel(pkt):
    if pkt.getDataType() == PyPacket.PacketDataType.PKT_RF_STACKED_MAP_MSG:
        # log received rf model msg
        return True
    else:
        return False


def isStateInfo(pkt):
    if pkt.getDataType() == PyPacket.PacketDataType.PKT_AUTOPILOT_PIXHAWK:
        return True
    else:
        return False


def parseStackedMapMsg(data, nodes_to_learn, x_grids, y_grids):
    # update the models for the nodes that we have
    msg = PyPackets_pb2.RF_Stacked_Map_Msg
    msg.ParseFromString(data)

    # Get the data sets so we can run the planning algorithm
    My_Maps = {}
    this_map = {}
    for map in msg.mapMsg:
        # loop through and grab all the cells in the map
        for c in map.cell:
            x = c.xgridNum
            y = c.ygridNum
            this_map[x, y] = c.est_path_loss
        # use the map id to store into MyMaps
        My_Maps[map.ID] = this_map
        # clear the map
        del this_map

    # TODO! Fix the inputs
    # Run the waypoint calculation between desired nodes
    best_location = find_best_rf_position(My_Maps, nodes_to_learn, x_grids, y_grids)

    # Hand it to something to be made into a mission
    # TODO! figure out the pixhawk shit (maybe return to main loop? serial though is in the sensor loop)


def find_best_rf_position(maps, nodes, x, y):
    # if the map contains the node key (how do we check this in python?)
    if len(nodes) == 2:
        i = 0
        location = find_optimal_2_node_location(maps[nodes[i]], maps[nodes[i + 1]], x, y)
        return location
    else:
        print 'Error in Number of Nodes; cannot find best location'
        return None


def find_optimal_2_node_location(MapA, MapB, x, y):
    # loop through the maps
    storedCost = 9999
    storedLocation = [0, 0]
    # THIS DOESN'T WORK BECAUSE A,B is negative and positive i believe
    for a in x:
        for b in y:
            # calculate the cost
            results = MapA[a, b] + MapB[a, b] + math.fabs(MapA[a, b] - MapB[a, b])
            if storedCost > results:
                # store the new best cost
                storedCost = results
                # store their location
                storedLocation = [a, b]
            elif storedCost == results:
                print 'Multiple best locations found: Keeping previous results'
    # End for loops
    print "The best location was found to have a cost of %f" % storedCost
    return storedLocation


# Parse the command from the GCS
def parseCommand(data):
    # Determine the information in the message
    pass
    # Set the mode of operation
    # -Passive
    # -Autonomous


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
    # end loop


def buildLearnCommand(counter, NodeListStrings):
    pkt_id = PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT, 10)
    pkt = PyPacket.PyPacket()
    pkt.setDataType(PyPacket.PacketDataType.PKT_RF_LEARN_CMD)
    pkt.setID(pkt_id.getBytes())
    msg = PyPackets_pb2.RF_Learn_Cmd()
    msg.packetNum = counter
    msg.ID = str(pkt.getID())
    msg.time = time.time()
    for string in NodeListStrings:
        new = msg.NodesToLearn.add()
        new = string
    # End for loop
    data_str = msg.SerializeToString()
    pkt.setData(data_str)
    del msg
    return pkt.getPacket()


# TODO FIX THIS COPIED FUNCTION
# Build the Node Packet for Subscribers to let the network manager know what this process wants
def buildNodePacket(subscribers, PacketCounterNode):
    pkt_id = PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT, 10)
    pkt = PyPacket.PyPacket()
    pkt.setDataType(PyPacket.PacketDataType.PKT_NODE_HEARTBEAT)
    pkt.setID(pkt_id.getBytes())
    # Define the basic components
    msg = PyPackets_pb2.NodeHeartBeat()
    msg.packetNum = PacketCounterNode
    msg.ID = str(pkt.getID())
    msg.time = time.time()
    # Add all the subscriber infos
    c = 0;
    for n in subscribers:
        new = msg.sub.add()
        new.id = str(n.ID)
        new.datatype = str(n.TYPE)
        new.port = n.PORT
        new.address = n.IP
        new.msgfreq = n.FREQ
        c += 1
    # End loop
    # serialize the data
    data_str = msg.SerializeToString()
    pkt.setData(data_str)  # normally insert a data building part
    # pkt.displayPacket()
    del msg
    return pkt.getPacket()  # return the byte array msg


'''
To communicate sensor measurements and learned fields to the ground station for monitoring
Uses Network Manager Process
'''


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
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_STACKED_MAP_MSG,
                                                  PyPacket.PacketID(sub_info_rf[0], sub_info_rf[1]).getBytes(),
                                                  self.PORT, 'localhost', 1))  # RF Model from GP Task
        # STATE MSG (Could be from a simulated source or not)
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_AUTOPILOT_PIXHAWK,
                                                  PyPacket.PacketID(sub_info_state[0], sub_info_state[1]).getBytes(),
                                                  self.PORT, 'localhost', 1))

        self.logger.debug("Finished initializing telemetry task")

    def run(self):
        # Create Socket Objects
        my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        my_in_socket.bind(('', self.PORT))

        # Create lists for select
        in_sockets = [my_in_socket]
        out_sockets = [my_out_socket]

        # Add subscriber message to msg queue to be sent
        msg = buildNodePacket(self.sublist, 0)  # do we want to send this at set intervals?
        msg_queue.put(msg)
        lastsubtime = time.time()

        # Loop Forever
        while not shutdown_event.is_set():
            # Here is where we would put messages that send data at set intervals
            if (time.time() - lastsubtime) > 10:
                lastsubtime = time.time()
                # Should we update the packetnumber?
                msg_queue.put(msg)

            # TODO! Add in a cmd to ask for a learn message

            # Check select
            readable, writable, exceptional = select.select(in_sockets, out_sockets, in_sockets)

            for s in readable:
                # get data from socket
                dataPkt, address = s.recvfrom(PyPacket.RECVBUFF)
                # assign datapkt as an object
                newPkt = PyPacket.PyPacket()
                newPkt.setPacket(dataPkt)
                # Parse packet
                if isCommand(newPkt):
                    self.logger.debug("Received Command Msg")
                    parseCommand(newPkt.getData())
                elif isLearnedModel(newPkt):
                    self.logger.debug("Received Learned Model Msg")
                    parseStackedMapMsg(newPkt.getData(), planning_nodes, self.grid_x_points, self.grid_y_points)
                elif isStateInfo(newPkt):  # used for simulating the aircraft's position
                    self.logger.debug("Received State Info Msg: Sim")
                    parseStateInfo(newPkt.getData(), self.CLLA)
                else:
                    # Unexpected message
                    self.logger.warning("Msg Parsing Problem: %s", 'Unexpected message type')
            # END

            for s in writable:
                # Check to see if our output queue is empty
                try:
                    next_msg = msg_queue.get_nowait()
                except Queue.Empty:
                    # queue is empty or blocked
                    time.sleep(0.01)
                else:
                    s.sendto(next_msg, ('localhost', self.NMPORT))  # should always be localhost:NMport
                    self.logger.info("Message sent to: %s", ('localhost', self.NMPORT))
        # End While Loop

        my_out_socket.close()
        my_in_socket.close()

        self.logger.info("CommAware Telemetry Task [Closed]")
        print('\tTelemtryTask [closed]')


class SensingTask(threading.Thread):
    def __init__(self, antenna_names, altitude, grid_info, grid_info_2, logmode, center_pos, my_id, mode):
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
        self.logger.info("Logging Sensor Packets to: ", self.packet_log.logname)

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
            for b in range(0, int(self.ygridlength)-1):
                for a in range(0, int(self.xgridlength)-1):
                    #TODO! Same error as before; look at learning app to fix
                    # set the byte number
                    byteNumber = Splat_Processing.findPLinFile(a, b, self.xgridlength-1)  # Is it minus 1??
                    self.logger.debug("seeking to file at %i", byteNumber)
                    file.seek(byteNumber)
                    bytesIn = file.read(4)
                    thisfloat = struct.unpack('f', bytesIn)
                    self.logger.debug("Retrieved float from file: %f at %i, %i", thisfloat[0], a, b)
                    thisMap[a, b] = thisfloat
                    secondMap[a, b] = 0
            # End for loop
            # Store into the main list of maps
            self.MyNodeMaps[name] = thisMap
            self.MeasuredNodeMaps[name] = secondMap
            # also need to store the AntennaGains at some point into the system
            # TODO! store antenna gains from file that we read in RoI?
        # End antenna file loop

        # If simulation
        self.SensorMode = mode  # 0 = simulation, 1 = real time, 2 = test
        self.RF_SimModel = RF_Models.Simple_RF_Model(1)  # load the simulation model here
        self.transmitter = {}
        self.transmitter["NodeA"] = RF_Models.RF_Transmitter(3,3,2400000000,[40.147011,-105.241987])
        self.transmitter["NodeB"] = RF_Models.RF_Transmitter(3,3,2400000000,[40.138018,-105.244482])
        # TODO! load the antenna file information from SPLAT!
        self.logger.info("Set RF Sim Model to Noise Model 1")

        # Serial link?

    def run(self):
        global myState
        # Build the data type
        rf_data_msg = PyPackets_pb2.RF_Data_Msg()
        rf_data_msg.ID = str(self.MYID.getBytes())

        # Build the packet
        newPacket = PyPacket.PyPacket()
        newPacket.setDataType(PyPacket.PacketDataType.PKT_RF_DATA_MSG)
        newPacket.setID(self.MYID.getBytes())

        # Sensor rate
        sensor_rate = 1  # Set to 5Hz #TODO! where should we set this?
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
                    if myState: #TODO! Fix to a different check; is empty?
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
                                                               self.xgridlength)
                        # -------Y----------
                        ygrid = Splat_Processing.findGridIndex(myState[0], self.yspacing, self.ycenterIndex,
                                                               self.ygridlength)

                        # RF Part
                        # TODO! Update with new scheme for naming of nodes; list of nodes to measure/loop through them
                        for s in measuring_nodes:
                            self.logger.debug("Generating data for %s", node_name)
                            node_name = s
                            # Get the predicted path loss value
                            map = self.MyNodeMaps[node_name]
                            pl_predicted = map[xgrid, ygrid]
                            # create a new rf measurement for the message
                            new = rf_data_msg.rfNode.add()
                            new.chanID = node_name
                            new.rssi = -150  # place holder

                            new.rssi = float(self.RF_SimModel.generateMeasurement(self.transmitter[s], [myState[1],myState[2],-myState[5]], self.ReceiverGains)) #Transmitter, ENU Position
                            # Calculate the measured path loss using the antenna gains (no antenna pointing assumed)
                            new.pl_msr = float(-(new.rssi - self.AntennaGains[node_name] - self.ReceiverGains))
                            # figure out what the prediction error is
                            new.pl_prediction_error = float(pl_predicted - new.pl_msr)
                            # add in the grid location
                            new.xgridNum = xgrid
                            new.ygridNum = ygrid
                        # end loop

                        # time
                        rf_data_msg.time = time.time()
                        rf_data_msg.packetNum = rf_msg_counter
                        # increment counter
                        rf_msg_counter = rf_msg_counter + 1

                        # TODO! Add in packet logger

                        # Log
                        self.logger.info("RF Data Msg number %i with time value: %f", rf_data_msg.packetNum,
                                         rf_data_msg.time)

                        # TODO! Erroring as a result of not filling message as a result of an Else throw
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
                    # TODO! Add in the serial portion of the project (check Katie's code)
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
                                               [self.centerLat, self.centerLon, self.refAlt])
                    # -------X----------
                    xgrid = Splat_Processing.findGridIndex(ENU[0], self.xspacing, self.xcenterIndex, self.xgridlength)
                    # -------Y----------
                    ygrid = Splat_Processing.findGridIndex(ENU[1], self.yspacing, self.ycenterIndex, self.ygridlength)

                    # RF data part
                    # TODO! UPDATE WITH NEW PATHLOSS PARTS
                    try:
                        # TODO fix serial
                        line = ser.readline()
                        entries = line.split(",")
                        counter = 0
                        for s in range(0, len(entries)):
                            if entries[s].startswith('C'):
                                # ==============================
                                # ASSUMING CH1 == NodeA, etc....
                                # ==============================
                                # THIS MIGHT NEED TO BE FIXED; validate with hardware
                                rf_chan = entries[s]
                                rf_data = entries[s + 1], entries[s + 2]  # TODO! which is omni which is directional?
                                rf_sensor_chan.append(rf_chan)
                                rf_sensor_data.append(rf_data)

                                node_name = measuring_nodes[counter]
                                map = self.MyNodeMaps[node_name]
                                pl_predicted = map[xgrid, ygrid]

                                new = rf_data_msg.rfNode.add()
                                new.chanID = node_name
                                # calculate the "measured path loss"
                                # TODO! Update with new method of getting channel gains
                                msred_pl = -(rf_data[0] - self.AntennaGains[node_name] - self.ReceiverGains)
                                # the error between measured and predicted
                                pl_prediction_error = pl_predicted - msred_pl

                                new.rssi = float(rf_data[0])
                                new.pl_msr = float(msred_pl)
                                new.pl_error = float(pl_prediction_error)
                                new.xgridNum = xgrid
                                new.ygridNum = ygrid
                                # increment counter for stuffing into node stuff
                                counter += 1
                            else:
                                pass

                        # time
                        rf_data_msg.time = time.time()
                        rf_data_msg.packetNum = rf_msg_counter
                        # increment counter
                        rf_msg_counter = rf_msg_counter + 1
                        #TODO! Add in packet logger

                        # Log
                        self.logger.info("RF Data Msg number %i with time value: %f", rf_data_msg.packetNum,
                                         rf_data_msg.time)

                        # TODO! Erroring as a result of not filling message as a result of an Else throw
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

                    # TODO! Update / verify the chanID being used is appropriate for scheme in learning alg.
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

                    # TODO! Erroring as a result of not filling message as a result of an Else throw
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


def loadMetaData(filename):
    meta_data_f = open(filename, 'r')
    # Skip the defintions
    for v in xrange(0, 9):
        meta_data_f.readline()  # remove the header info
    # Read the RoI info
    centerLat = float(meta_data_f.readline())  # cast as float
    centerLon = float(meta_data_f.readline())  # cast as float
    north = float(meta_data_f.readline())
    south = float(meta_data_f.readline())
    east = float(meta_data_f.readline())
    west = float(meta_data_f.readline())
    xspacing = float(meta_data_f.readline())
    yspacing = float(meta_data_f.readline())
    # Get the antenna information
    Antenna = []
    for line in meta_data_f:
        print(line)
        Antenna.append(line)

    return centerLat, centerLon, north, south, east, west, xspacing, yspacing, Antenna


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
    parser.add_argument('NODES', type=str)  # list like NodeA,NodeB
    # Nodes to plan on

    # Parse the argument information
    args = parser.parse_args()
    # Handle the information to correct variables
    RoI_FILE = args.ROI_FILE
    ALTITUDE = args.ALTITUDE
    IP = 'localhost'
    PORT = 14300
    # Loop through and add all of these into the list somehow: CHECK OUT ARGPARSE PYTHON DOCS
    planning_nodes.append(args.NODES[0])

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
    LOGMODE = 10

    # Load Meta Data from Region File
    meta_data = loadMetaData(RoI_FILE)
    # mainlogger.critical("Couldn't find and open RoI File")

    # parse the meta data, everything else will be grabbed when needed
    # Calculate the X and Y Grid lengths
    x_grid_length = (meta_data[4] + meta_data[5]) / meta_data[6] + 1
    y_grid_length = (meta_data[2] + meta_data[3]) / meta_data[7] + 1
    # center index
    x_center_Index = math.ceil(x_grid_length / 2)
    y_center_Index = math.ceil(y_grid_length / 2)

    # GridInfo = [-west, east, xspacing, -south, north, yspacing]
    Grid_Info = [-meta_data[5], meta_data[4], meta_data[6], -meta_data[3], meta_data[2], meta_data[7]]
    # GridInfo2 = [x_grid_length,y_grid_length,x_center_index,y_center_index]
    Grid_Info_2 = [x_grid_length, y_grid_length, x_center_Index, y_center_Index]
    # centerPoint = [lat,lon]
    center_Point = [meta_data[0], meta_data[1]]
    # Pull out antenna information
    Antenna = meta_data[8]
    # Pull out the antenna names
    Antenna_Names = []
    strvalues = []
    for ant in Antenna:
        strvalues = ant.split(",")
        Antenna_Names.append(strvalues[0])
        # TODO! Anything else to retrieve from this bits of information?

    MODE_FLAG = 0  # 0 is sim, 1 is run, 2 is test

    if MODE_FLAG == 1:
        from dronekit import connect

        connection_string = '/dev/ttyS1'
        # connection_string = 'udp:0.0.0.0:14551'
        print 'Connecting to vehicle on: %s' % (connection_string)
        vehicle = connect(connection_string, baud=57600, _initialize=True, wait_ready=None)

    # =================================
    # Start Telemetry Thread
    telem = TelemetryTask(MY_ID, SUB_INFO_RF, SUB_INFO_GCS, SUB_INFO_STATE, PORT, IP, LOGMODE, Grid_Info, center_Point)
    telem.start()
    # Start Sensing Thread
    sensing = SensingTask(Antenna_Names, ALTITUDE, Grid_Info, Grid_Info_2, LOGMODE, center_Point, MY_ID, MODE_FLAG)
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
