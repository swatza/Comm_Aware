#Imports
from multiprocessing import Process
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
from sklearn.gaussian_process.kernels import Matern, WhiteKernel, ConstantKernel

sys.path.insert(0, '../PyUAS')
sys.path.insert(0, '../PyUAS/protobuf')
import PyPacket
import PyPackets_pb2
import PyPacketLogger
import Subscriber
import assorted_lib
import Splat_Processing as SplatProcessing

# Global Variables
NodeValues = {}
NodeCounts = {}
NodeModel = {}
TheseNodes = []

# process variable
p = None
#global msg queue
msg_queue = Queue.Queue()
#SHutdown event for threads
shutdown_event = threading.Event()
#kernel for GP work
Kernel = Matern(length_scale=25, nu=5 / 2) + ConstantKernel() + WhiteKernel(noise_level=1)


def isRFMsg(packet):
    if packet.getDataType() == PyPacket.PacketDataType.PKT_RF_DATA_MSG:
        return True
    else:
        return False

def isRFLearnCmd(packet):
    if packet.getDataType() == PyPacket.PacketDataType.PKT_RF_LEARN_CMD:
        return True
    else:
        return False

# TODO! Update this function for new way of managing data
def handleRFMsg(packetData):
    # we have just received a packets data
    datastr = str(packetData)
    # assign it to the google protobuf type
    rf_msg = PyPackets_pb2.RF_Data_Msg()
    rf_msg.ParseFromString(datastr)
    # get the relevant information out of the data structure
    for n in rf_msg.rfNode:
        node_name = n.chanID
        a = n.xgridNum
        b = n.ygridNum
        #Stick these values in the global set
        NodeCounts[node_name][a,b] += 1
        #Debug message?
        # TODO! Update so that we can also learn the whole field instead of just the error model
        NodeValues[node_name][a,b] += n.pl_error
        #Debug Message?
    return

#Update with new passing of data and stuff
def handleRFLearnMsg(my_id, packetData, counter,grid):
    datastr = str(packetData)
    # assign to the correct google protobuf type
    rf_learn_cmd = PyPackets_pb2.RF_Learn_Cmd()
    rf_learn_cmd.ParseFromString(datastr)
    #Generate pypacket
    thisPacket = PyPacket.PyPacket()
    thisPacket.setDataType(PyPacket.PacketDataType.PKT_RF_STACKED_MAP_MSG)
    thisPacket.setID(my_id)
    # Generate the msg
    BigMsg = PyPackets_pb2.RF_Stacked_Map_Msg()

    #Figure out what type of useful information is stored in here
    nodes = rf_learn_cmd.NodesToLearn
    processList = []
    processIndex = 0
    #Loop through the command to start a process for each node to learn
    for strN in nodes:
        # check to see if this node exists
        if isInNodeList(strN):
            # go ahead and spin off a process
            msg = BigMsg.mapMsg.add()
            # Arguments: msgNode,thisNodeString,thisNodeValues,thisNodeCounts,myModelPrediction,packetCo
            processList.append(Process(target=learnNode,args=(msg,strN,NodeValues[strN],NodeCounts[strN],NodeModel[strN],counter,grid)))
            processList[processIndex].start()
            #Increment the process index
            processIndex += 1
        else:
            print "Node is not available to learn"
    # Wait for all the processes to rejoin
    for p in processList:
        p.join()

    # Finalize packet setup
    ds = BigMsg.SerializeToString()
    thisPacket.setData(ds)

    #Add the packet to the msg_queue
    msg_queue.put(thisPacket.getPacket())
    return

def isInNodeList(this_node):
    #Loop through all the nodes that we have
    for node in TheseNodes:
        if node == this_node:
            return True
    return False

# TODO! Move this to a library file since we use it a lot
def buildNodeHBPacket(subscribers, Packet_Node_Counter, MY_ID):
    pkt = PyPacket.PyPacket()
    pkt.setDataType(PyPacket.PacketDataType.PKT_NODE_HEARTBEAT)
    pkt.setID(MY_ID)
    # Define the basic components
    msg = PyPackets_pb2.NodeHeartBeat()
    msg.packetNum = Packet_Node_Counter
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

def learnNode(msgPart, thisNodeString,thisNodeValues,thisNodeCounts,myModelPrediction,packetCounter,grid_info):
    # Purpose is to learn the error for a node
    # GridInfo = [-west, east, xspacing, -south, north, yspacing]
    xmin = grid_info[0]
    xmax = grid_info[1]
    ymin = grid_info[3]
    ymax = grid_info[4]
    xspacing = grid_info[2]
    yspacing = grid_info[5]

    # Set the prediction
    xpred = []
    grid_x_points = np.arange(xmin,xmax+1,xspacing)
    grid_y_points = np.arange(ymin,ymax+1,yspacing)

    for a in range(0,len(grid_x_points)):
        for b in range(0,len(grid_y_points)):
            xpred.append([grid_x_points[a],grid_y_points[b]])
    # Add the information about this node to the big msg
    msgPart.ID = thisNodeString
    msgPart.xGrids = len(grid_x_points)
    msgPart.yGrids = len(grid_y_points)
    msgPart.xspacing = xspacing
    msgPart.yspacing = yspacing

    # transform the list to an array
    Xprediction = np.array(xpred) # Is this still needed

    # Create the data set
    Xdata = []
    Ydata = []
    for a in range(0,len(grid_x_points)):
        for b in range(0,len(grid_y_points)):
            Xdata.append([a,b])
            Ydata.append(thisNodeValues[a,b]/thisNodeCounts[a,b])
    # End creation of data set
    results = GP_Processing(Xdata,Ydata,grid_x_points,grid_y_points,xpred)
    ttL = results[0]
    ttP = results[1]
    Z = results[2]

    # Combine the learned errors with the original predictions???
    E = []
    for a in range(0, len(grid_x_points)):
        for b in range(0,len(grid_y_points)):
            E[a,b] = Z[a,b] + myModelPrediction[a,b]
            newCell = msgPart.cell.add()
            newCell.xgridNum = a
            newCell.ygridNum = b
            newCell.est_path_loss = E[a,b]
            newCell.path_loss_err = Z[a,b]
            newCell.pred_path_loss = myModelPrediction[a,b]
    # End combined errors
    msgPart.time = time.time()
    msgPart.packetNum = packetCounter
    msgPart.iteration_number = packetCounter
    msgPart.gp_learning_time = ttL
    msgPart.gp_prediction_time = ttP

    #End
    return

def GP_Processing(Xdata,Ydata,grid_x_points, grid_y_points,xpred):
    #Create GP tool
    gptool = gaussian_process.GaussianProcessRegressor(kernel=Kernel)

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

class TelemetryTask(threading.Thread):
    def __init__(self,MYID,SUBINFO,myport,Logmode,gridinformation):
        threading.Thread.__init__(self)

        #Need a logger
        self.logger = logging.getLogger("LearningApp:TelemetryTask")
        self.logger.setLevel(Logmode)
        myHandler = logging.StreamHandler()
        self.logger.addHandler(myHandler)

        # ID and Comm information
        self.MYPORT = myport
        self.NMPORT = 16000
        self.IP = 'localhost'
        self.ID = str(MYID)

        #Grid Information to be passed on
        self.grid = gridinformation

        # Add the subscribers we need
        self.sublist = []
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_DATA_MSG,
                                                  PyPacket.PacketID(SUBINFO[0],SUBINFO[1]).getBytes(),
                                                  self.MYPORT, self.IP, 1))  # RF Data Msg from sensor part

    def run(self):
        my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        my_in_socket.bind(('', self.MYPORT))

        #Learning variables to track
        p = None
        missedcounter = 0
        node_packet_counter = 0
        counter = 0

        # Simple timers for status output
        prev_time = 0

        # Create lists for select
        in_sockets = [my_in_socket]
        out_sockets = [my_out_socket]

        # Add subscriber message to msg queue to be sent
        msg = buildNodeHBPacket(self.sublist, node_packet_counter, self.ID)  # do we want to send this at set intervals?
        msg_queue.put(msg)

        print 'Starting While Loop'
        # Loop Forever
        while not shutdown_event.is_set():
            # Check select
            readable, writable, exceptional = select.select(in_sockets, out_sockets, in_sockets)

            tlast_since_status = time.time() - prev_time
            # Print out statement just saying we are waiting for mesages
            if tlast_since_status > 10:
                msg_queue.put(msg)
                print 'Waiting for messages'
                prev_time = time.time()

            for s in readable:
                # get data from socket
                dataPkt, address = s.recvfrom(PyPacket.RECVBUFF)
                # assign datapkt as an object
                newPkt = PyPacket.PyPacket()
                newPkt.setPacket(dataPkt)
                # Parse packet
                if isRFMsg(newPkt):
                    print 'Received new RF Data'
                    handleRFMsg(newPkt.getData())
                elif isRFLearnCmd(newPkt):
                    print 'Received a Learn Command'
                    # TODO! What??
                    if p is not None:
                        if p.is_alive():
                            #Don't start a new command yet
                            print "Learning is still running: Missed a Command"
                            missedcounter += 1
                        else:
                            p = Process(handleRFLearnMsg,args(self.ID,newPkt.getData(),counter,self.grid))
                            counter += 1
                    else:
                        p = Process(handleRFLearnMsg,args(self.ID,newPkt.getData(),self.grid))
                        counter += 1
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
                    self.logger.info("Message sent to: %s", ('localhost', self.NMPORT))
        # End While Loop

        if p is not None:
            if p.is_alive():
                # wait for the process to join before quiting
                p.join()
        my_out_socket.close()
        my_in_socket.close()

        print('\tTelemtryTask [closed]')

def loadMetaData(filename):
    meta_data_f = open(RoI_File, 'r')
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

    return centerLat,centerLon,north,south,east,west,xspacing,yspacing,Antenna

if __name__ == "__main__":
    #Parse Arguments
    parser = argparse.ArgumentParser(description='Comm-Aware Autonomous Application for aircraft')
    parser.add_argument('PLATFORM',type=str)
    parser.add_argument('SYSTEM_NUMBER', type=int)
    parser.add_argument('TARGET_PLATFORM', type=str)
    parser.add_argument('TARGET_NUMBER', type=int)
    parser.add_argument('ROI_FILE', type=str)
    parser.add_argument('LOG_LEVEL',type=int)

    args = parser.parse_args()

    # TODO! Initialize a logger for main thread

    #Get the meta data file name
    RoI_File = args.ROI_FILE

    #figure out the log mode level: for now just set as debug
    LOG_MODE = logging.DEBUG

    #Determine my port
    MY_PORT = 14310 #TODO add as an optional argument

    #Parse this programs ID information (Platform and Number)
    SYSTEM_PLATFORM = PyPacket.PlatformDispatch[PyPacket.formatPlatformString(args.PLATFORM)]()
    SYSTEM_NUMBER = args.SYSTEM_NUMBER
    MY_ID = PyPacket.PacketID(SYSTEM_PLATFORM,SYSTEM_NUMBER)

    #Parse this programs ID information for the subscriber (Platform and Number)
    TARGET_PLATFORM = PyPacket.PlatformDispatch[PyPacket.formatPlatformString(args.TARGET_PLATFORM)]()
    TARGET_NUMBER = args.TARGET_NUMBER
    SUB_INFO = [TARGET_PLATFORM,TARGET_NUMBER]
    # TODO! Extend this to multiple aircraft/sensor stations

    # Load Meta Data from Region File
    meta_data = loadMetaData(RoI_File)

    # parse the meta data, everything else will be grabbed when needed
    # Calculate the X and Y Grid lengths
    x_grid_length = (meta_data[4] + meta_data[5]) / meta_data[6] + 1
    y_grid_length = (meta_data[2] + meta_data[3]) / meta_data[7] + 1
    # center index
    x_center_Index = math.ceil(x_grid_length / 2)
    y_center_Index = math.ceil(y_grid_length / 2)

    # GridInfo = [-west, east, xspacing, -south, north, yspacing]
    Grid_Info = [-meta_data[5],meta_data[4],meta_data[6],-meta_data[3],meta_data[2],meta_data[7]]
    # centerPoint = [lat,lon]
    center_Point = [meta_data[0], meta_data[1]]
    #Pull out antenna information
    Antenna = meta_data[8]

    # Get the antenna locations or file names
    antenna_splat_filenames = []
    strvalues = []
    for ant in Antenna:
        strvalues = ant.split(",")
        TheseNodes.append(strvalues[0])
        antenna_splat_filenames.append(strvalues[0] + ".ppm")
        #Initialize these values; we will zero them next
        NodeCounts[strvalues[0]] = {}
        NodeValues[strvalues[0]] = {}
        NodeModel[strvalues[0]] = {}

    #Using the files generate the predicted model data
    file_counter = 0
    for f in antenna_splat_filenames:
        file = open(f,'r')
        node_name = TheseNodes[file_counter]
        TempModel = {}
        TempCounter = {}
        TempValue = {}
        for b in range(0, int(y_grid_length)):
            for a in range(0, int(x_grid_length)):
                #Set the byte number
                byteNumber = SplatProcessing.findPLinFile(a,b,x_grid_length)
                #seek to location in file and read 4 bytes (float)
                file.seek(byteNumber)
                bytesIn = file.read(4)
                #Unpack the binary data
                thisfloat = struct.unpack('f',bytesIn)
                #Add this to the correct model
                TempModel[a,b] = thisfloat
                #Zero the counter and other parts
                TempCounter[a,b] = 0
                TempValue[a,b] = 0
        #End x-y loop
        NodeModel[node_name] = TempModel
        NodeValues[node_name] = TempValue
        NodeCounts[node_name] = TempCounter
        #increment file counter
        file_counter += 1
    #End file loop

    telem = TelemetryTask(MY_ID,SUB_INFO,MY_PORT,LOG_MODE,Grid_Info)
    telem.start()

    while threading.active_count() > 1:
        try:
            time.sleep(.5)
        except(KeyboardInterrupt, SystemExit):
            shutdown_event.set()
    #end while
    sys.exit()
    #Make sure to force join threads
#End Main Loop
