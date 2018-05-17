# Imports
from multiprocessing import Pool
import select
import time
import Queue
import sys
import argparse
import numpy as np
import math
import logging
import struct
import socket
import threading
from sklearn import gaussian_process
from sklearn.gaussian_process.kernels import Matern, WhiteKernel, ConstantKernel, RBF

sys.path.insert(0, '../PyUAS')  # get the path to the PyUAS Folder
sys.path.insert(0, '../PyUAS/protobuf')  # get the path to the protobuf format
import PyPacket
import PyPackets_pb2
import PyPacketLogger
import Subscriber
import assorted_lib

NodeA = {}  # Sum of all the pl error values
NodeA_Ct = {}  # number of values being summed
NodeB = {}
NodeB_Ct = {}
NodeC = {}
NodeC_Ct = {}
numberOfMeasurements = 0
msg_queue = Queue.Queue()
shutdown_event = threading.Event()



def handleRFMsg(packetData):
    # we have just received a packets data
    datastr = str(packetData)
    # assign it to the google protobuf type
    rf_msg = PyPackets_pb2.RF_Data_Msg()
    rf_msg.ParseFromString(datastr)
    # get the relevant information out of the data structure
    # Get grid location (Use function we will put in splat processing)

    # ignoring altitudes
    counter = 0
    for n in rf_msg.rfNode:
        a = n.xgridNum
        b = n.ygridNum
        if counter == 0:
            NodeA[a, b] = n.pl_error
            NodeA_Ct[a, b] += 1
            counter += 1
        elif counter == 1:
            NodeB[a, b] = n.pl_error
            NodeB_Ct[a, b] += 1
            counter += 1
        elif counter == 2:
            NodeC[a, b] = n.pl_error
            NodeC_Ct[a, b] += 1
            counter += 1

    # Increment count of number of measurements we have added to each dictionary/map
    numberOfMeasurements += 1
    return


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
    c = 0
    for n in subscribers:
        new = msg.sub.add()
        new.id = str(n.ID)
        new.datatype = str(n.TYPE)
        new.port = n.PORT
        new.address = n.IP
        new.msgfreq = 1 #hardcoded for now
        c += 1
    # End loop
    # serialize the data
    data_str = msg.SerializeToString()
    pkt.setData(data_str)  # normally insert a data building part
    pkt.displayPacket()
    del msg
    return pkt.getPacket()  # return the byte array msg

def findBestRFPosition(FieldData):
    storedPoint = [9999, 9999]  # x,y grids
    costAtPoint = 999999
    for a in range(0, xgridsize):
        for b in range(0, ygridsize):
            if FieldData[a, b] < costAtPoint:
                costAtPoint = FieldData[a, b]
                storedPoint[0] = a
                storedPoint[1] = b
            # do we want a case if they are equal?
            elif FieldData[a, b] == costAtPoint:
                print 'Found a point of equal value'
    # else:
    # it is not a better cost; move along

    return storedPoint, costAtPoint


def GP_Processing(gptool,Xdata, Ydata, grid_x_points, grid_y_points):
    # Finished getting data; set the dtype
    X = np.array(Xdata, dtype=np.float32)
    Y = np.array(Ydata, dtype=np.float32)

    # learn the hyper parameters
    gpStart = time.time()
    gptool.fit(X, Y)
    gpFinished = time.time()
    timeToLearn = gpFinished - gpStart

    # predicting with the learned model
    gpStart = time.time()
    y_pred, sigma = gptool.predict(xpred, return_std=True)
    gpFinished = time.time()
    timeToPredict = gpFinished - gpStart

    # Convert to mesh with Predicted value for a given X,Y Coord
    Z = np.zeros((len(grid_x_points), len(grid_y_points)))
    counter = 0

    for a in range(0, len(grid_x_points)):
        for b in range(0, len(grid_y_points)):
            Z[a, b] = y_pred[counter]
            counter = counter + 1

    # Return something useful
    return timeToLearn, timeToPredict, Z

# TODO FINISH THIS TASK
class TelemetryTask(threading.Thread):
    def __init__(self, aircraftNumber, myport, Logmode):
        threading.Thread.__init__(self)

        #Need a logger
        self.logger = logging.getLogger("LearningApp:TelemetryTask")
        self.logger.setLevel(Logmode)
        myhandler = logging.StreamHandler()
        self.logger.addHandler(myhandler)

        # stuff
        self.MYPORT = myport
        self.NMPORT = 16000
        self.ip = 'localhost'
        self.Num = aircraftNumber

        self.sublist = []
        # RF_DATA_MSG
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_DATA_MSG,
                                               PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT, self.Num).getBytes(),
                                               self.MYPORT, 'localhost',1))  # RF Data Msg from sensor part

    def run(self):
        my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        my_in_socket.bind(('', self.MYPORT))

        # Create lists for select
        in_sockets = [my_in_socket]
        out_sockets = [my_out_socket]

        # Add subscriber message to msg queue to be sent
        msg = buildNodePacket(self.sublist, 0)  # do we want to send this at set intervals?
        msg_queue.put(msg)

        # Loop Forever
        while not shutdown_event.is_set():
            # Check select
            readable, writable, exceptional = select.select(in_sockets, out_sockets, in_sockets)

            for s in readable:
                # get data from socket
                dataPkt, address = s.recvfrom(RECV_BUFF)
                # assign datapkt as an object
                newPkt = PyPacket.PyPacket()
                newPkt.setPacket(dataPkt)
                # Parse packet
                if isRFMsg(newPkt):
                    handleRFMsg(newPkt.getData)
                else:
                    # Unexpected message
                    print 'Unexpected Message'
            # END

            for s in writable:
                # Check to see if our output queue is empty
                try:
                    next_msg = msg_queue.get_nowait()
                except Queue.Empty:
                    # queue is empty or blocked
                    time.sleep(0.01)
                else:
                    s.sendto(next_msg, ('localhost', self.NMPORT))  # Should always be localhost at 16000
        # End While Loop

        my_out_socket.close()
        my_in_socket.close()

        print('\tTelemtryTask [closed]')


# TODO Finish the creation of the data in the Task startup
class GaussianProcessTask(threading.Thread):
    def __init__(self, Logmode, GridInfo, centerPoint, planningMode, filenames,aircraftnumber):
        threading.Thread.__init__(self)

        # Logger
        self.logger = logging.getLogger("CommAwareLearning:GaussianProcessTask")
        self.logger.setLevel(Logmode)
        myhandler = logging.StreamHandler()
        self.logger.addHandler(myhandler)

        self.packet_log = PyPacketLogger.PyPacketLogger(('Aircraft_' + str(aircraftnumber) + '_GP_Task_Log'))
        self.packet_log.initFile()
        self.logger.info("Logging RF Map Packets to: %s", self.packet_log.logname)

        # Initialize GP information (Check out my gp task from earlier versions)
        self.Kernel = Matern(length_scale=25, nu=5 / 2) + ConstantKernel() + WhiteKernel(noise_level=1)


        self.id = str(PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,aircraftnumber).getBytes())

        # Either by File here or elsewhere?
        self.xmin = GridInfo[0]
        self.xmax = GridInfo[1]
        self.ymin = GridInfo[3]
        self.ymax = GridInfo[4]
        self.xspacing = GridInfo[2]
        self.yspacing = GridInfo[5]

        self.centerLat = centerPoint[0]
        self.centerLon = centerPoint[1]

        # Load the propagation model into memory from file
        if (len(filenames) == 1):
            self.model_NodeA = {}
            self.NumberOfNodes = 1
        elif (len(filenames) == 2):
            self.model_NodeA = {}
            self.model_NodeB = {}
            self.model_NumberOfNodes = 2
        elif (len(filenames) > 2):
            self.model_NodeA = {}
            self.model_NodeB = {}
            self.model_NodeC = {}
            self.NumberOfNodes = 3
        counter = 0
        for f in filenames:
            # find the file
            file = open(f, 'r')
            for b in range(0, int(ygridlength)):
                for a in range(0, int(xgridlength)):
                    #Replace this part with the function call from Splat_Processing
                    # set the byte number
                    byteNumber = findPLinFile(a,b,xgridlength)
                    # Seek to location in file
                    file.seek(byteNumber)
                    # read float
                    bytesIn = file.read(4)  # read 4 bytes
                    # unpack into float?
                    thisfloat = struct.unpack('f', bytesIn)
                    if counter == 0:
                    # add it to Node A
                        self.model_NodeA[a, b] = thisfloat
                    elif counter == 1 and self.NumberOfNodes > 1:
                    # add it to Node B
                        self.model_NodeB[a, b] = thisfloat
                    elif counter == 2 and self.NumberOfNodes > 2:
                    # add it to Node C
                        self.model_NodeC[a, b] = thisfloat
                    # End loop through grids
                    counter += 1
                    # end for loop

                    self.mode = planningMode  # some sort of string

    def run(self):

        # Prediction Zone for the GP
        xpred = []
        grid_x_points = np.arange(self.xmin, self.xmax + 1, self.xspacing)
        grid_y_points = np.arange(self.ymin, self.ymax + 1, self.yspacing)

        for a in range(0, len(grid_x_points)):
            for b in range(0, len(grid_y_points)):
                xpred.append([grid_x_points[a], grid_y_points[b]])

        # Create the messages for Protobuf
        BigMsg = PyPackets_pb2.RF_STACKED_MAP_MSG()
        msgA = BigMsg.mapMsg.add()
        if self.NumberOfNodes > 1:
            msgB = BigMsg.mapMsg.add()
            msgB.ID = 'NodeB'
            msgB.xGrids = len(grid_x_points)
            msgB.yGrids = len(grid_y_points)
            msgB.xspacing = self.xspacing
            msgB.yspacing = self.yspacing
        if self.NumberOfNodes > 2:
            msgC = BigMsg.mapMsg.add()
            msgC.ID = 'NodeC'
            msgC.xGrids = len(grid_x_points)
            msgC.yGrids = len(grid_y_points)
            msgC.xspacing = self.xspacing
            msgC.yspacing = self.yspacing

        # THIS IS GONNA CAUSE AN ISSUE WITH OUR CURRENT MSG HANDLING SYSTEM!!!
        msgA.ID = 'NodeA'
        msgA.xGrids = len(grid_x_points)
        msgA.yGrids = len(grid_y_points)
        msgA.xspacing = self.xspacing
        msgA.yspacing = self.yspacing

        WPMsg = PyPackets_pb2.Waypoint()

        # Set the gaussian process
        gp = gaussian_process.GaussianProcessRegressor(kernel=self.Kernel)
        #DO i need 3 different gaussian processes?
        gp2 = gaussian_process.GaussianProcessRegressor(kernel=self.Kernel)
        gp3 = gaussian_process.GaussianProcessRegressor(kernel=self.Kernel)

        # STACKED MAP MSG PACKET
        pkt_id = PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT, self.Number)
        thisPacket = PyPackets.PyPacket()
        thisPacket.setDataType(PyPacket.PacketDataType.PKT_RF_STACKED_MAP_MSG)
        thisPacket.setID(pkt_id.getBytes())

        # WAYPOINT PACKET
        pkt_id = PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT, self.Number)
        thatPacket = PyPackets.PyPacket()
        thatPacket.setDataType(PyPacket.PacketDataType.PKT_WAYPOINT)
        thatPacket.setID(pkt_id.getBytes())

        # transform the list to an array
        Xprediction = np.array(xpred)

        GPTimeLast = time.time()
        GPTimeNow = GPTimeLast()
        GPRate = 60  # at least this amount of time (60s)
        packetNum = 0  # start at 0

        Na = {}
        Nb = {}
        Nc = {}

        # Multiple worker processes for the GP Learning part
        p = Pool(3)

        # Loop
        while not shutdown_event.is_set():
            if time.time() - GPTimeLast >= GPRate:

                # Initialize empty arrays
                Xdata = []
                # Add in control based on the number of nodes
                YdataNodeA = []
                YdataNodeB = []
                YdataNodeC = []

                # Process data
                for a in range(0, xgridsize):
                    for b in range(0, ygridsize):
                        # calculate the x data and ydata for each of the GPs
                        Xdata.append([a, b])
                        # Add in control checks based on number of nodes
                        YdataNodeA.append(NodeA[a, b] / NodeA_Ct[a, b])
                        YdataNodeB.append(NodeB[a, b] / NodeB_Ct[a, b])
                        YdataNodeC.append(NodeC[a, b] / NodeC_Ct[a, b])

                # Spin off 3 different "processes/multitasks" for each of the GPs with their associated data
                #Add control depending on many nodes we have
                results = p.map(GP_Processing, [[gp, Xdata, YdataNodeA, grid_x_points, grid_y_points],
                                                [gp2, Xdata, YdataNodeB, grid_x_points, grid_y_points],
                                                [gp3, Xdata, YdataNodeC, grid_x_points, grid_y_points]])

                # Pull out the useful information from the results
                NAR = results[0]
                NBR = results[1]
                NCR = results[2]
                ttLA = NAR[0]
                ttPA = NAR[1]
                ZnA = NAR[2]
                ttLB = NBR[0]
                ttPB = NBR[1]
                ZnB = NBR[2]
                ttLC = NCR[0]
                ttPC = NCR[1]
                ZnC = NCR[2]

                # Combine the learned errors with the original prediction
                for a in range(0, xgridsize):
                    for b in range(0, ygridsize):
                        #Check if multiple sets exist
                        Na[a, b] = ZnA[a, b] + self.model_NodeA[a, b]
                        newCellA = msgA.cell.add()
                        newCellA.xgridNum = a
                        newCellA.ygridNum = b
                        newCellA.est_path_loss = Na[a, b]
                        newCellA.path_loss_err = ZnA[a, b]
                        newCellA.pred_path_loss = self.model_NodeA[a, b]

                        if self.NumberOfNodes > 1:
                            Nb[a, b] = ZnB[a, b] + self.model_NodeB[a, b]
                            newCellB = msgB.cell.add()
                            newCellB.xgridNum = a
                            newCellB.ygridNum = b
                            newCellB.est_path_loss = Nb[a, b]
                            newCellB.path_loss_err = ZnB[a, b]
                            newCellB.pred_path_loss = self.model_NodeB[a, b]

                        if self.NumberOfNodes > 2:
                            Nc[a, b] = ZnC[a, b] + self.model_NodeC[a, b]
                            newCellC = msgC.cell.add()
                            newCellC.xgridNum = a
                            newCellC.ygridNum = b
                            newCellC.est_path_loss = Nc[a, b]
                            newCellC.path_loss_err = ZnC[a, b]
                            newCellC.pred_path_loss = self.model_NodeC[a, b]

                # Increment packet Number
                packetNum += 1

                # Store the rest of the msg data
                msgA.time = time.time()
                msgA.packetNum = packetNum
                msgA.gp_iteration_number = packetNum
                msgA.gp_learning_time = ttLA
                msgA.gp_prediction_time = ttPA

                if self.NumberOfNodes > 1:
                    msgB.time = time.time()
                    msgB.packetNum = packetNum
                    msgB.gp_iteration_number = packetNum
                    msgB.gp_learning_time = ttLB
                    msgB.gp_prediction_time = ttPB

                if self.NumberOfNodes > 2:
                    msgC.time = time.time()
                    msgC.packetNum = packetNum
                    msgC.gp_iteration_number = packetNum
                    msgC.gp_learning_time = ttLC
                    msgC.gp_prediction_time = ttPC
                # Delete the repeated msg types
                # TODO!

                # Go ahead and add thisPacket to the queue
                ds = BigMsg.SerializeToString()
                thisPacket.setData(ds)
                # Stuff into the great que
                msg_queue.put(thisPacket.getPacket())

                # Next is planning!
                # which mode is being used;
                # what two nodes are we looking at?
                # set the cost function inputs based on which two nodes we are looking at
                Data = costfunction(Na, Nb)
                gridPoints, cost = findBestRFPosition(Data)
                # Some how get LLA?
                E = grid_x_points[gridPoints[0]]
                N = grid_y_points[gridPoints[1]]
                LLA = assorted_lib.ENU2LLA([E, N, 0], [self.centerLat, self.centerLon])

                #Set teh output message
                WPMsg.LLA_Pos.x = LLA[0]
                WPMsg.LLA_Pos.y = LLA[1]
                WPMsg.LLA_Pos.z = LLA[2]
                WPMsg.cost = cost
                WPMsg.costF1 = Na[gridPoints]
                WpMsg.costF2 = Nb[gridPoints]
                # Go ahead and thatPacket to the que
                ds = WPMsg.SerializeToString()
                thatPacket.setData(ds)
                msg_queue.put(thatPacket.getPacket())
        # End of if loop
        # End of While loop
        time.sleep(1)  # sleep


# end of function

if __name__ == "__main__":
    # we need the prediction map stored
    # we need to write the cost function calculation method
    # Scenario 1: 2 Beacons and flying between them.
    # Scenario 2: 3 Beacons and rotating between flying between 2 of them at any given moment
    # - always learn all 3 fields at the same time? do we want to spin off a multi-task thingy
    parser = argparse.ArgumentParser(description='Comm-Aware Autonomous Application for aircraft')
    parser.add_argument('AIRCRAFT_NUMBER', type=int)
    parser.add_argument('ROI_FILE', type=str)

    args = parser.parse_args()
    RoI_FILE = args.ROI_FILE
    aircraftNumber = args.AIRCRAFT_NUMBER

    # Any Arguments???
    # -ROI File Name
    # -Planning Mode or is that from a telem cmd stream?
    gcsNumber = 0

    # Load from file
    meta_data_f = open(RoI_FILE, 'r')
    # Skip the defintions
    for a in xrange(0, 9):
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

    # Get the antenna locations or file names
    antenna_names = []
    strvalues = []
    for ant in Antenna:
        strvalues = ant.split(",")
        antenna_names.append(strvalues[0]+".ppm")  # add all the string valued names

    # Calculate the X and Y Grid lengths
    xgridlength = (east + west) / xspacing + 1
    ygridlength = (north + south) / yspacing + 1
    # center index
    xcenterIndex = math.ceil(xgridlength / 2)
    ycenterIndex = math.ceil(ygridlength / 2)

    GridInfo = [-west, east, xspacing, -south, north, yspacing]
    centerPoint = [centerLat, centerLon]
    planningMode = "SomeStringTODO"

    Logmode = logging.DEBUG  # need to make my own logger or use their own?
    Ipaddress = 'localhost'

    # Start Telemetry Thread
    telem = TelemetryTask(aircraftNumber, 14130, Logmode)
    telem.start()
    # Start Sensing Thread
    modelling = GaussianProcessTask(Logmode, GridInfo, centerPoint, planningMode, antenna_names,aircraftNumber)
    modelling.start()

    while threading.active_count() > 1:
        try:
            time.sleep(1)
        except(KeyboardInterrupt, SystemExit):
            # log message
            shutdown_event.set()
    # end try
    # end while
    # log ending message
    sys.exit()
# END MAIN LOOP
