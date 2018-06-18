#Imports
from multiprocessing import Process, Queue, Lock
import multiprocessing
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
from sklearn.gaussian_process.kernels import Matern, WhiteKernel, ConstantKernel, ExpSineSquared

sys.path.insert(0, '../PyUAS')
sys.path.insert(0, '../PyUAS/protobuf')
import PyPacket
import PyPackets_pb2
import PyPacketMsgBuilds
import PyPacketTypeCheck
import PyPacketLogger
import Subscriber
import assorted_lib
import Splat_Processing as SplatProcessing
import comm_aware_control_tools

# Global Variables
NodeValues = {}
NodeCounts = {}
NodeModel = {}
TheseNodes = []

# process variable
p = None
#global msg queue
msg_queue = multiprocessing.Queue() #multiprocessing version
#SHutdown event for threads
shutdown_event = threading.Event()
#kernel for GP work
Kernel = Matern(length_scale=25, nu=5 / 2) + ConstantKernel() + WhiteKernel(noise_level=1)
#Kernel = ExpSineSquared() + WhiteKernel(1e-1)


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
        # TODO! Update so that we can also learn the whole field instead of just the error model??
        NodeValues[node_name][a,b] += n.pl_error
        #Debug Message?
    return


#Update with new passing of data and stuff
def handleRFLearnMsg(my_id, packetData, counter,grid,msg_queue,l,node_counts, node_values,node_model):
    #create a packet logger
    packet_log = PyPacketLogger.PyPacketLogger(('Learning_App_' + my_id.getPlatform() + str(my_id.getIdentifier()) + "_Log"))
    packet_log.initFile()
    l.acquire()
    print "Starting RF Learning"
    l.release()
    datastr = str(packetData)
    # assign to the correct google protobuf type
    rf_learn_cmd = PyPackets_pb2.RF_Learn_Cmd()
    rf_learn_cmd.ParseFromString(datastr)
    #Generate pypackets for sending / recording data
    thisPacket = PyPacket.PyPacket()
    thisPacket.setDataType(PyPacket.PacketDataType.PKT_RF_STACKED_MAP_MSG)
    thisPacket.setID(my_id.getBytes())
    wpt_packet = PyPacket.PyPacket()
    wpt_packet.setDataType(PyPacket.PacketDataType.PKT_WAYPOINT)
    wpt_packet.setID(my_id.getBytes())
    # Generate the msg
    BigMsg = PyPackets_pb2.RF_Stacked_Map_Msg()

    #Figure out what type of useful information is stored in here
    nodes = rf_learn_cmd.NodesToLearn
    l.acquire()
    print nodes
    l.release()
    processList = []
    pipelist = []
    msglist = []
    processIndex = 0
    #Loop through the command to start a process for each node to learn
    for strN in nodes:
        # check to see if this node exists
        l.acquire()
        print ("Going to learn " + strN)
        l.release()
        if isInNodeList(strN):
            l.acquire()
            print "It exists in our node list"
            l.release()
            # go ahead and spin off a process and a pipe
            recv_end, send_end = multiprocessing.Pipe(False)
            msg = BigMsg.mapMsg.add()
            msglist.append(msg)
            # Arguments: msgNode,thisNodeString,thisNodeValues,thisNodeCounts,myModelPrediction,packetCo
            processList.append(Process(target=learnNode,args=(msg,strN,node_values[strN],node_counts[strN],node_model[strN],counter,grid,l,send_end)))
            pipelist.append(recv_end)
            processList[processIndex].start()
            #Increment the process index
            processIndex += 1
        else:
            l.acquire()
            print "Node is not available to learn"
            l.release()
    # Wait for all the processes to rejoin
    for i in range(0, 2): #try hardcoded 2?
        msglist[i] = pipelist[i].recv()
    #some how set these to the emssages
    l.acquire()
    print "We have handed off the messages"
    l.release()
    for i in range(0,len(msglist)): #len(msglist)
        BigMsg.mapMsg[i].ID = msglist[i].ID
        BigMsg.mapMsg[i].xGrids = msglist[i].xGrids
        BigMsg.mapMsg[i].yGrids = msglist[i].yGrids
        BigMsg.mapMsg[i].xSpacing = msglist[i].xSpacing
        BigMsg.mapMsg[i].ySpacing = msglist[i].ySpacing
        BigMsg.mapMsg[i].time = msglist[i].time
        BigMsg.mapMsg[i].packetNum = msglist[i].packetNum
        BigMsg.mapMsg[i].gp_iteration_number = msglist[i].gp_iteration_number
        BigMsg.mapMsg[i].gp_learning_time = msglist[i].gp_learning_time
        BigMsg.mapMsg[i].gp_prediction_time = msglist[i].gp_prediction_time
        l.acquire()
        print "Looping for each cell"
        l.release()
        for k in range(0,len(msglist[i].cell)):
            BigMsg.mapMsg[i].cell.add()
            BigMsg.mapMsg[i].cell[k].xgridNum = msglist[i].cell[k].xgridNum
            BigMsg.mapMsg[i].cell[k].ygridNum = msglist[i].cell[k].ygridNum
            BigMsg.mapMsg[i].cell[k].est_path_loss = msglist[i].cell[k].est_path_loss
            BigMsg.mapMsg[i].cell[k].path_loss_err = msglist[i].cell[k].path_loss_err
            BigMsg.mapMsg[i].cell[k].pred_path_loss = msglist[i].cell[k].pred_path_loss
        #End
    #End

        #doesn't like this part; perhaps manually assign each of the values?

    for p in processList:
        p.join()
    #put the msgs together into the bigmsg? lets see if this works
    l.acquire()
    print "We have merged the processes"
    l.release()

    # Finalize packet setup
    ds = BigMsg.SerializeToString()
    thisPacket.setData(ds)
    packet_log.writePacketToLog(thisPacket)

    #Determine if we are going to be doiing a waypoint claculation
    nodepoints = rf_learn_cmd.NodesToPlan
    if rf_learn_cmd.calculateWaypointFlag == "True":
        wpt_msg = PyPackets_pb2.Waypoint()
        #Calculate the grids
        xmin = grid[0]
        xmax = grid[1]
        ymin = grid[3]
        ymax = grid[4]
        xspacing = grid[2]
        yspacing = grid[5]

        x_grids = np.arange(xmin, xmax, xspacing)
        y_grids = np.arange(ymin, ymax, yspacing)

        #Build Two maps
        MyMaps = {}
        for map in BigMsg.mapMsg:
            this_map = {}
            #loop through and grab all the cells in the map
            for c in map.cell:
                x = c.xgridNum
                y = c.ygridNum
                this_map[x,y] = c.est_path_loss
            #end cell loop
            MyMaps[map.ID] = this_map
            del this_map
        #End map loop
        #Calculate the waypoint between the nodes we want
        location = comm_aware_control_tools.find_optimal_2_node_location(MyMaps,nodepoints,len(x_grids),len(y_grids))
        l.acquire()
        print "We have found the optimal location to fly to in ENU Coordinates"
        l.release()
        #build the waypoint message (Not in LLA
        wpt_msg.Pos.x = x_grids[location[0]]
        wpt_msg.Pos.y = y_grids[location[1]]
        wpt_msg.Pos.z = 0
        wpt_msg.frame = "ENU"
        wds = wpt_msg.SerializeToString()
        #Log to pypacket logger
        wpt_packet.setData(wds)
        packet_log.writePacketToLog(wpt_packet)
        #Add to msg que
        msg_queue.put(wpt_packet.getPacket())
    #Add the packet to the msg_queue
    return


def isInNodeList(this_node):
    #Loop through all the nodes that we have
    for node in TheseNodes:
        if node == this_node:
            return True
    return False


def learnNode(msgPart, thisNodeString,thisNodeValues,thisNodeCounts,myModelPrediction,packetCounter,grid_info,mylock,send_end_pipe):
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
    grid_x_points = np.arange(xmin,xmax,xspacing)
    grid_y_points = np.arange(ymin,ymax,yspacing)

    mylock.acquire()
    print "The length of the axes are: %i, %i" %(len(grid_x_points), len(grid_y_points))
    mylock.release()

    for a in range(0,len(grid_x_points)):
        for b in range(0,len(grid_y_points)):
            xpred.append([grid_x_points[a],grid_y_points[b]])

    # Add the information about this node to the big msg
    msgPart.ID = thisNodeString
    msgPart.xGrids = len(grid_x_points)
    msgPart.yGrids = len(grid_y_points)
    msgPart.xSpacing = int(round(xspacing))
    msgPart.ySpacing = int(round(yspacing))

    # transform the list to an array
    mylock.acquire()
    print "About to loop through and generate the y values"
    mylock.release()

    # Create the data set
    Xdata = []
    Ydata = []
    for a in range(0,len(grid_x_points)):
        for b in range(0,len(grid_y_points)):
            x = grid_x_points[a]
            y = grid_y_points[b]
            Xdata.append([x, y])
            if thisNodeCounts[a, b] == 0:
                Ydata.append(0)
            else:
                Ydata.append(thisNodeValues[a, b] / thisNodeCounts[a, b])
    # End creation of data set
    mylock.acquire()
    print "About to start GP"
    mylock.release()

    results = GP_Processing(Xdata,Ydata,grid_x_points,grid_y_points,xpred)
    ttL = results[0]
    ttP = results[1]
    Z = results[2] #Also a map of {a,b] indexes to value

    mylock.acquire()
    print ("Finished GP requiring %f : %f" %(ttL,ttP))
    mylock.release()

    # Combine the learned errors with the original predictions???
    E = {}
    for a in range(0,len(grid_x_points)):
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
    msgPart.gp_iteration_number = packetCounter
    msgPart.gp_learning_time = ttL
    msgPart.gp_prediction_time = ttP

    #End
    mylock.acquire()
    print ("Sending the output back to the main process")
    mylock.release()
    send_end_pipe.send(msgPart)
    #send_end_pipe.close()
    mylock.acquire()
    print ("Pipe Closed")
    mylock.release()
    return

def GP_Processing(Xdata,Ydata,grid_x_points, grid_y_points,xpred):
    #Create GP tool
    gptool = gaussian_process.GaussianProcessRegressor(kernel=Kernel,optimizer='fmin_l_bfgs_b')
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
        self.ID = MYID

        #Grid Information to be passed on
        self.grid = gridinformation

        # Add the subscribers we need
        self.sublist = []
        #RF Data
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_DATA_MSG,
                                                  PyPacket.PacketID(SUBINFO[0],SUBINFO[1]).getBytes(),
                                                  self.MYPORT, self.IP, 1))  # RF Data Msg from sensor part
        #RF Learn Command
        self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_LEARN_CMD, PyPacket.PacketID(SUB_INFO[0],SUBINFO[1]).getBytes(), self.MYPORT,self.IP,1))

    def run(self):
        my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        my_in_socket.bind(('', self.MYPORT))

        #Learning variables to track
        lock = multiprocessing.Lock()
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
        msg = PyPacketMsgBuilds.buildNodeHeartBeat(self.ID, self.sublist, node_packet_counter)
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
                if PyPacketTypeCheck.isRFDataMsg(newPkt):
                    print 'Received new RF Data'
                    handleRFMsg(newPkt.getData())
                elif PyPacketTypeCheck.isRFLearnCmd(newPkt):
                    print 'Received a Learn Command'
                    # TODO! What??
                    if p is not None:
                        print 'P is not none'
                        if p.is_alive():
                            #Don't start a new command yet
                            print "Learning is still running: Missed a Command"
                            missedcounter += 1
                        else:
                            p = Process(target=handleRFLearnMsg,args=(self.ID, newPkt.getData(), counter, self.grid, msg_queue, lock, NodeCounts, NodeValues, NodeModel))
                            p.start()
                            counter += 1
                    else:
                        p = Process(target=handleRFLearnMsg,args=(self.ID, newPkt.getData(), counter, self.grid, msg_queue, lock, NodeCounts, NodeValues, NodeModel))
                        p.start()
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

    logger = logging.getLogger("LearningApp:Startup")
    logger.setLevel(LOG_MODE)
    myHandler = logging.StreamHandler()
    logger.addHandler(myHandler)

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
    #TODO! PRobably don't need now!
    grid_x_points = np.arange(Grid_Info[0], Grid_Info[1], Grid_Info[2])
    grid_y_points = np.arange(Grid_Info[3], Grid_Info[4], Grid_Info[5])

    # Get the antenna locations or file names
    antenna_splat_filenames = []
    strvalues = []
    for ant in Antenna:
        strvalues = ant.split(",")
        TheseNodes.append(strvalues[0])
        antenna_splat_filenames.append(strvalues[0] + "_data_100.sdg")
        #Initialize these values; we will zero them next
        NodeCounts[strvalues[0]] = {}
        NodeValues[strvalues[0]] = {}
        NodeModel[strvalues[0]] = {}

    #Using the files generate the predicted model data
    file_counter = 0
    for f in antenna_splat_filenames:
        file = open(f,"r")
        node_name = TheseNodes[file_counter] #TODO! WHAT IS THESENODES
        TempModel = {}
        TempCounter = {}
        TempValue = {}
        for b in range(0, int(y_grid_length-1)):
            for a in range(0, int(x_grid_length-1)):
                #THERE IS A BUG HERE
                #Set the byte number
                byteNumber = SplatProcessing.findPLinFile(a,b,x_grid_length-1)
                logger.debug("Seeking to file at %i", byteNumber)
                #seek to location in file and read 4 bytes (float)
                file.seek(byteNumber)
                bytesIn = file.read(4)
                #Unpack the binary data
                thisfloat = struct.unpack('f',bytesIn)
                logger.debug("Retrieved float from file %f at %i, %i", thisfloat[0],a,b)
                #Add this to the correct model
                TempModel[a,b] = thisfloat[0]
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
