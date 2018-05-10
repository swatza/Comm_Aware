'''
Aircraft Communication Aware Application

Collects sensor measurements from a RF sensor (emulated or real), logs this information, calculates the error from a
prediction model and stores this. Uses this data to learn the error using a Gaussian Process and then predict
to create an estimated path loss field

Author: Spencer Watza
Copyright: CU Boulder RECUV

'''

#IMPORTS
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

sys.path.insert(0,'../PyUAS') #get the path to the PyUAS Folder
sys.path.insert(0,'../PyUAS/protobuf') #get the path to the protobuf format
import PyPacket
import PyPackets_pb2
import Subscriber
import RF_Models
import assorted_lib
import PyPacketLogger
import Splat_Processing

#SHutdown event
shutdown_event = threading.Event()

#Outgoing message que
msg_queue = Queue.Queue()

#My Aircraft State
myState = []

#Determine if this message is a command message
def isCommand(pkt):
	if pkt.getDataType() == PyPacket.PacketDataType.PKT_GCS_CMD_MSG:
		#log received gcs cmd message
		return True
	else:
		return False

#Determine if this message is the learned model message
def isLearnedModel(pkt):
	if pkt.getDataType() == PyPacket.PacketDataType.PKT_RF_MODEL_MSG:
		#log received rf model msg
		return True
	else:
		return False
		
def isStateInfo(pkt):
	if pkt.getDataType() == PyPacket.PacketDataType.PKT_AUTOPILOT_PIXHAWK:
		return True
	else:
		return False
		
def isWaypoint(pkt):
	if pkt.getDataType() == PyPacket.PacketDataType.PKT_WAYPOINT:
		return True
	else:
		return False

#Parse the command from the GCS
def parseCommand(data):
	#Determine the information in the message
	pass
	#Set the mode of operation
	#-Passive
	#-Autonomous 
	
#Parse the learned model from the GP Task
def parseLearnedModel(data):
	pass 
	#Do we even care about this function? 
	
	#Build a waypoint command message to send to pixhawk
	#TODO! See Andrew's Code he sent me
	
	#If in autonomous planning mode
		#Send the waypoint cmd to pixhawk
		
	#send the desired output waypoint command message to the GCS
	
	#end loop

def parseStateInfo(data, CLLA):
	#create the state information
	msg = PyPackets_pb2.AircraftPixhawkState()
	msg.ParseFromString(data)
	
	#Should we just leave it as Lat/Lon for pn,pe?
	NED = LLA2NED([msg.LLA_Pos[0],msg.LLA_Pos[1],msg.LLA_Pos[2]],CLLA)
	#myState[0] #pn
	#myState[1] #pe
	#myState[3] #yaw
	#myState[5] #height
	#myState[6] #Airspeed
	#Set the state as 1A Kinematics for now
	mystate[0] = NED[0]
	mystate[1] = NED[1]
	mystate[2] = 0 #yaw rate not needed
	mystate[3] = msg.attitude.z #yaw
	mystate[4] = 0 #height change not needed
	mystate[5] = NED[2]
	mystate[6] = msg.airspeed
	#end loop
	
def parseBestWaypoint(data):
	#parse the waypoint Lat,Lon,Altitude
	msg = PyPackets_pb2.Waypoint()
	msg.ParseFromString(data)
	#create a waypoint in ardupilot
	WLat = msg.LLA_Pos.x
	WLon = msg.LLA_Pos.y
	Alt = msg.LLA_Pos.z #is this right? 
	#Cost
	cost = msg.cost
	costFA = msg.costF1
	costFB = msg.costF2

#TODO FIX THIS COPIED FUNCTION
#Build the Node Packet for Subscribers to let the network manager know what this process wants
def buildNodePacket(subscribers, PacketCounterNode): 
	pkt_id = PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,10)
	pkt = PyPacket.PyPacket()
	pkt.setDataType(PyPacket.PacketDataType.PKT_NODE_HEARTBEAT)
	pkt.setID(pkt_id.getBytes())
	#Define the basic components
	msg = PyPackets_pb2.NodeHeartBeat()
	msg.packetNum = PacketCounterNode
	msg.ID = str(pkt.getID())
	msg.time = time.time()
	#Add all the subscriber infos
	c = 0;
	for n in subscribers:
		new = msg.sub.add()
		new.id = str(n.ID)
		new.datatype = str(n.TYPE)
		new.port = n.PORT
		new.address = n.IP
		new.msgfreq = n.FREQ
		c += 1
	#End loop
	#serialize the data
	data_str = msg.SerializeToString()
	pkt.setData(data_str) #normally insert a data building part
	#pkt.displayPacket()
	del msg
	return pkt.getPacket() #return the byte array msg


'''
To communicate sensor measurements and learned fields to the ground station for monitoring
Uses Network Manager Process
'''
class TelemetryTask(threading.Thread):

	def __init__(self, aircraftNumber, PortNumber, Ipaddress, gcsNumber, Logmode, GCSPOS):
		threading.Thread.__init__(self)
		
		#create logger
		self.logger = logging.getLogger("CommAwareApp:TelemetryTask")
		self.logger.setLevel(Logmode)
		myhandler = logging.StreamHandler()
		self.logger.addHandler(myhandler)
		self.logger.info("CommAware Telemetry Task Started")
		
		self.CenterLLA = GCSPOS
		
		#network setup
		self.PORT = PortNumber
		self.IP = 'localhost'
		self.ID = 'AC' #not needed?
		self.Num = 10
		self.GCS_Num = 1
		self.NMPORT = 16000
		d = {"port": self.PORT, "ip": self.IP, "id": self.ID, "Number": self.Num, "GCS": self.GCS_Num, "NMPort": self.NMPORT}
		self.logger.debug("Initialized Values for the Telemetry: %s",' ',extra=d)
		
		#Generate a list of subscribers Here
		self.sublist = []
		#GCS COMMAND MSG
		self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_GCS_CMD_MSG, PyPacket.PacketID(PyPacket.PacketPlatform.GROUND_CONTROL_STATION,self.GCS_Num).getBytes(),self.PORT,'localhost',1)) #GCS Command Msgs
		#RF LEARNED MODEL
		self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_STACKED_MAP_MSG, PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,self.Num).getBytes(),self.PORT,'localhost',1)) #RF Model from GP Task
		#PKT WAYPOINT 
		self.sublist.append(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_WAYPOINT, PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,self.Num).getBytes(),self.PORT,'localhost',1)) #RF Model from GP Task
		
		self.logger.debug("Finished initializing telemetry task")
		
		
	def run(self):
		#Create Socket Objects
		my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		
		my_in_socket.bind(('',self.PORT))
		
		#Create lists for select
		in_sockets = [my_in_socket]
		out_sockets = [my_out_socket]
		
		#Add subscriber message to msg queue to be sent
		msg = buildNodePacket(self.sublist,0) #do we want to send this at set intervals?
		msg_queue.put(msg)
		lastsubtime = time.time()
		
		#Loop Forever
		while not shutdown_event.is_set():
			#Here is where we would put messages that send data at set intervals
			if (time.time() - lastsubtime) > 10:
				lastsubtime = time.time()
				#Should we update the packetnumber?
				msg_queue.put(msg)
			
			#Check select
			readable, writable, exceptional = select.select(in_sockets,out_sockets,in_sockets)
			
			for s in readable:
				#get data from socket
				dataPkt, address = s.recvfrom(RECV_BUFF)
				#assign datapkt as an object
				newPkt = PyPacket.PyPacket()
				newPkt.setPacket(dataPkt)
				#Parse packet
				if isCommand(newPkt):
					self.logger.debug("Received Command Msg")
					parseCommand(newPkt.getData)
				elif isLearnedModel(newPkt):
					parseLearnedModel(newPkt.getData)
					self.logger.debug("Received Learned Model Msg")
				elif isStateInfo(newPkt): #used for simulating the aircraft's position
					parseStateInfo(newPkt.getData, self.CenterLLA)
					self.logger.debug("Received State Info Msg: Sim")
				elif isWaypoint(newPkt):
					parseBestWaypoint(newPkt.getData)
					self.logger.debug("Received Waypoint Msg")
				else:
					#Unexpected message 
					self.logger.warning("Msg Parsing Problem: %s", 'Unexpected message type')
			#END 
			
			for s in writable:
				#Check to see if our output queue is empty
				try:
					next_msg = msg_queue.get_nowait()
				except Queue.Empty:
					#queue is empty or blocked
					time.sleep(0.01)
				else:
					s.sendto(next_msg,('localhost',self.NMPORT)) #should always be localhost:NMport
					self.logger.info("Message sent to: %s", ('localhost',self.NMPORT))
		#End While Loop
		
		my_out_socket.close()
		my_in_socket.close()
		
		self.logger.info("CommAware Telemetry Task [Closed]")
		print('\tTelemtryTask [closed]')
		
class SensingTask(threading.Thread):
	def __init__(self,antennaNames,altitude,ygridlength,xgridlength,xcind,ycind,xspace,yspace,Logmode,centerPos,aircraftid):
		threading.Thread.__init__(self)
		
		#create logger
		self.logger = logging.getLogger("CommAwareApp:SensingTask")
		self.logger.setLevel(Logmode)
		myhandler = logging.StreamHandler()
		self.logger.addHandler(myhandler)
		self.logger.info("CommAware Sensing Task Started")
		
		self.packet_log = PyPacketLogger.PyPacketLogger( ('Aircraft_' + str(aircraftid)+ '_Sensing_Task_Log'))
		self.packet_log.initFile()
		self.logger.info("Logging Sensor Packets to: ", self.packet_log.logname)
		
		#important parameters
		self.center = centerPos 
		self.refAlt = 0
		#Grab these from the antenna file?
		self.tx_power = 10 #dBm 
		self.tx_gains = 3 #dB
		self.rx_gain = 3 #dB
		
		self.AIRCRAFT_ID = aircraftid
		
		self.xcenterIndex = xcind
		self.ycenterIndex = ycind
		self.xspacing = xspace
		self.yspacing = yspace
		self.xgridlength = xgridlength
		self.ygridlength = ygridlength
		
		#Load the propagation model into memory from file
		if (len(antennaNames) == 1):
			self.NodeA = {}
			self.NumberOfNodes = 1
			self.logger.info("1 Antenna Node Being Used")
		elif (len(antennaNames) == 2):
			self.NodeA = {}
			self.NodeB = {}
			self.NumberOfNodes = 2
			self.logger.info("2 Antenna Nodes Being Used")
		elif (len(antennaNames) > 2):
			self.NodeA = {}
			self.NodeB = {}
			self.NodeC = {}
			self.NumberOfNodes = 3
			self.logger.info("3 Antenna Nodes Being Used")
		counter = 0
		for name in antennaNames:
			#find the file
			file = open(name + "_data_" + altitude + ".sdg", "r")
			for b in range(0,int(ygridlength-1)):
				for a in range(0,int(xgridlength-1)):
					#set the byte number
					byteNumber = Splat_Processing.findPLinFile(a,b,xgridlength-1)
					self.logger.debug("Seeking to file at %i", byteNumber)
					#Seek to location in file
					file.seek(byteNumber)
					#read float 
					bytesIn = file.read(4) #read 4 bytes
					#unpack into float? 
					thisfloat = struct.unpack('f',bytesIn)
					self.logger.debug("Retrieved float from file: %fat %i, %i", thisfloat[0],a,b) #at %i, %i",a,b)
					if counter == 0:
						#add it to Node A
						self.NodeA[a,b] = thisfloat
					elif counter == 1:
						#add it to Node B
						self.NodeB[a,b] = thisfloat
					elif counter == 2:
						#add it to Node C
						self.NodeC[a,b] = thisfloat
			#End loop through grids
			counter += 1
		#end for loop
		
		#If simulation
		self.SensorMode = 2 #0 = simulation, 1 = real time, 2 = test
		self.RF_SimModel = RF_Models.Simple_RF_Model(1) #load the simulation model here
		self.logger.info("Set RF Sim Model to Noise Model 1")
		
	def run(self):
		
		#Build the data type
		rf_data_msg = PyPackets_pb2.RF_Data_Msg()
		rf_data_msg.ID = str(PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,self.AIRCRAFT_ID).getBytes())
		
		#Build the packet
		newPacket = PyPacket.PyPacket()
		newPacket.setDataType(PyPacket.PacketDataType.PKT_RF_DATA_MSG)
		newPacket.setID(PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,self.AIRCRAFT_ID).getBytes())
		
		#Sensor rate
		sensor_rate = 1
		
		#Loop Forever
		while not shutdown_event.is_set():
			time_last = time.time()
			now_time = time.time()
			#sensor rate
			if (time_last - now_time) >= sensor_rate:
				rf_sensor_data = [] #initialize the list where i can store the multiple channels
				rf_sensor_chan = []
				#-----------------
				#Simulation Mode
				#-----------------
				if self.SensorMode == 0: 
					#if our state is not null
					if (myState != null):
				
						#if the state is 1A Kinematics Model
						#myState[0] #pn
						#myState[1] #pe
						#myState[3] #yaw
						#myState[5] #height
						#myState[6] #Airspeed
						
						#Convert NED to ENU
						LLA = assorted_lib.NED2LLA([myState[0],myState[1],myState[5]], self.center)
						
						rf_data_msg.lla.x = LLA[0]
						rf_data_msg.lla.y = LLA[1]
						rf_data_msg.lla.z = LLA[2]
						
						#optional Skip msg forms
						
						#Determine my location in the grid
						#-------X----------
						xgrid = findGridIndex(mystate[1],self.xspacing,self.xcenterIndex,self.xgridlength)
						#-------Y----------
						ygrid = findGridIndex(mystate[0],self.yspacing,self.ycenterIndex,self.ygridlength)
						
						#RF Part
						for s in range (0,self.NumberOfNodes):
							if s == 0:
								pl_predicted = self.NodeA[xgrid,ygrid]
							elif s == 1:
								pl_predicted = self.NodeB[xgrid,ygrid]
							elif s == 2:
								pl_predicted = self.NodeC[xgrid,ygrid]
							#Each Node 
							new = rf_dta_msg.rfNode.add()
							new.chanID = str(s) #which node is it
							new.rssi = float(self.RF_SimModel.generateMeasurement(self.RF_SimModel.transmitter[s],[myState[1],myState[2],-myState[5]])) #Transmitter, ENU Position
							#new.rssi = 0-random.uniform(60,100) #Eventually replaced with simulation model
							new.pl_msr = float(-(new.rssi - self.rx_gain - self.tx_gains - self.tx_power))
							#figure out what the prediction error is
							new.pl_prediction_error  = float(pl_predicted - new.pl_msr) #Place holder
							#add in the grid location so we don't look it up again
							new.xgridNum = xgrid
							new.ygridNum = ygrid
						#end loop
						
						#time
						rf_data_msg.time = time.time()
						rf_data_msg.packetNum = rf_msg_counter
						#increment counter
						rf_msg_counter = rf_msg_counter + 1
						
					else:
						print 'No state found'
					
				#-----------------
				#Normal Mode
				#-----------------
				elif self.SensorMode == 1:
					#Pull vehicle data using dronekit
					rf_data_msg.lla.x = vehicle.location.global_frame.lat
					rf_data_msg.lla.y = vehicle.location.global_frame.lon
					rf_data_msg.lla.z = vehicle.location.global_relative_frame.alt
					#Optional
					rf_data_msg.attitude.x = vehicle.attitude.roll
					rf_data_msg.attitude.y = vehicle.attitude.pitch
					rf_data_msg.attitude.z = vehicle.attitude.yaw
					rf_data_msg.airspeed = vehicle.airspeed
					
					#-First determine indexes
					ENU = lla2flatENU(rf_data_msg.lla.x,rf_data_msg.lla.y,rf_data_msg.lla.z,self.centerLat,self.centerLon,self.refAlt)
					#-------X----------
					xgrid = findGridIndex(ENU[0],self.xspacing,self.xcenterIndex,self.xgridlength)
					#-------Y----------
					ygrid = findGridIndex(ENU[1],self.yspacing,self.ycenterIndex,self.ygridlength)

					#RF data part
					#TODO! UPDATE WITH NEW PATHLOSS PARTS
					try:
						line = ser.readline()
						entries = line.split(",")
						for s in range (0,len(entries)):
							if entries[s].startswith('C'):
								#THIS MIGHT NEED TO BE FIXED; validate with hardware
								rf_chan = entries[s]
								rf_data = entries[s+1],entries[s+2]
								rf_sensor_chan.append(rf_chan)
								rf_sensor_data.append(rf_data)
								
								new = rf_data_msg.rfNode.add()
								new.chanID = rf_chan
								#calculate the "measured path loss"
								msred_pl = -(rf_data[0] - self.rx_gain - self.tx_gains - self.tx_power)
								#the error between measured and predicted
								pl_prediction_error = pl_predicted - msred_pl
								
								new.rssi = float(rf_data[0])
								new.pl_msr = float(msred_pl)
								new.pl_error = float(pl_prediction_error)
								new.xgridNum = xgridNum
								new.ygridNum = ygridNum
							else:
								pass
					
						#time
						rf_data_msg.time = time.time()
						rf_data_msg.packetNum = rf_msg_counter
						#increment counter
						rf_msg_counter = rf_msg_counter + 1
						
					except: 
						#log message
						print "serial error with sensors"
					#End try loop
				
				
				#-----------------
				#Test Mode
				#-----------------
				elif self.SensorMode == 2:
					#Test outputs with just hardcoded numbers for now
					rf_data_msg.packetNum = 999
					rf_data_msg.time = time.time()
					rf_data_msg.lla.x = 40.130692
					rf_data_msg.lla.y = -105.244599
					rf_data_msg.lla.z = 1700 #ASL ?
					rf_data_msg.attitude.x = 0
					rf_data_msg.attitude.y = 0
					rf_data_msg.attitude.z = 0
					rf_data_msg.airspeed = 15
					
					for ind in range(0,3):
						newnode = rf_data_msg.rfNode.add()
						if ind == 0:
							newnode.chanID = "CH1"
							newnode.rssi = -62
							newnode.pl_msr = 97
							newnode.pl_error = -12
						elif ind == 1:
							newnode.chanID = "CH2"
							newnode.rssi = -48
							newnode.pl_msr = 80
							newnode.pl_error = -4
						elif ind == 2:
							newnode.chanID = "CH3"
							newnode.rssi = -55
							newnode.pl_msr = 88
							newnode.pl_error = 9

						newnode.xgridNum = 5
						newnode.ygridNum = 10
					
				
				#-----------------
				#Unknown Mode
				#-----------------
				else:
					self.logger.critical("Unknown Mode Detected: No data is being generated. Quitting!")
					sys.exit()
				#END MODE LOOPS
				
				#Log
				self.logger.info("RF Data Msg number %i with time value: %f",(rf_data_msg.packetNum, rf_data_msg.time))
				
				#serialize 
				data_str = rf_data_msg.SerializeToString()
				
				#Put the data into a packet
				newPacket.setData(data_str)
				#log packet
				self.packet_log.writePacketToLog(newPacket)
				
				#Add the packet to the global send queue
				msg_queue.put(newPkt.getPacket())
				del rf_data_msg.rfNode[:]
			else:
				time.sleep(0.01)
			#END FREQ LOOPS
		#END WHILE
		
		self.logger.info("Closing Sensor Task")
		print('\t SensorTask [closed]')
			
	

if __name__  == "__main__":
	#Load a simulation file that has starting location and simulation model info
	mainlogger = logging.getLogger("CommAwareApp:Main")
	
	#Parse Arguments; For now assume defaults
	parser = argparse.ArgumentParser(description='Comm-Aware Autonomous Application for aircraft')
	parser.add_argument('AIRCRAFT_NUMBER',type=int)
	parser.add_argument('ROI_FILE',type=str)
	parser.add_argument('ALTITUDE',type=str)
	
	args = parser.parse_args()
	
	RoI_FILE = args.ROI_FILE
	AIRCRAFT_NUMBER = args.AIRCRAFT_NUMBER
	ALTITUDE = args.ALTITUDE
	IP = 'localhost'
	gcsNumber = 1
	
	#HARD coded debug mode for now
	LOGMODE = 10
	
	#Load from file
	meta_data_f = open(RoI_FILE,'r')
	if meta_data_f is None:
		mainlogger.critical("Couldn't find and open RoI File")
		
	#Skip the defintions
	for a in xrange(0,9):
		meta_data_f.readline() #remove the header info
	#Read the RoI info
	centerLat = float(meta_data_f.readline()) #cast as float
	centerLon = float(meta_data_f.readline()) #cast as float
	north = float(meta_data_f.readline()) 
	south = float(meta_data_f.readline())
	east = float(meta_data_f.readline())
	west = float(meta_data_f.readline())
	xspacing = float(meta_data_f.readline())
	yspacing = float(meta_data_f.readline())
	#Get the antenna information
	Antenna = []
	for line in meta_data_f:
		print(line)
		Antenna.append(line)
		
	mainlogger.info("Number of Antennas %i", len(Antenna))
	#Get the antenna locations or file names
	antenna_names = []
	strvalues = []
	for ant in Antenna:
		strvalues = ant.split(",")
		antenna_names.append(strvalues[0]) #add all the string valued names
	
	#NEEDED?
	GCSPosition = [centerLat,centerLon] #turn into cLat,cLon
	
	#Calculate the X and Y Grid lengths
	xgridlength = (east + west)/xspacing + 1
	ygridlength = (north + south)/yspacing + 1
	#center index
	xcenterInd = math.ceil(xgridlength/2);
	ycenterInd = math.ceil(ygridlength/2);

	#Start Telemetry Thread
	telem = TelemetryTask(AIRCRAFT_NUMBER, 14120, IP, gcsNumber, LOGMODE, GCSPosition)
	telem.start()
	#Start Sensing Thread
	sensing = SensingTask(antenna_names,ALTITUDE,ygridlength,xgridlength,xcenterInd,ycenterInd,xspacing,yspacing, LOGMODE, GCSPosition,AIRCRAFT_NUMBER)
	sensing.start()
	
	while threading.active_count() > 1:
		try:
			time.sleep(1)
		except(KeyboardInterrupt, SystemExit):
			#log message
			shutdown_event.set()
		#end try
	#end while
	#log ending message
	sys.exit()
	
	
	