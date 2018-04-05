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
import argparse

import PyPackets
import PyPackets_pb2
import Subscriber



#Outgoing message que
msg_queue = Queue.Queue()

#Determine if this message is a command message
def isCommand(pkt):
	if pkt.getDataType() == PyPacket.PacketDataType.PKT_GCS_CMD_MSG:
		#log received gcs cmd message
		return true
	else:
		return false

#Determine if this message is the learned model message
def isLearnedModel(pkt):
	if pkt.getDataType() == PyPacket.PacketDataType.PKT_RF_MODEL_MSG:
		#log received rf model msg
		return true
	else:
		return false

#Parse the command from the GCS
def parseCommand(data):
	#Determine the information in the message
	
	#Set the mode of operation
	#-Passive
	#-Autonomous 
	
#Parse the learned model from the GP Task
def parseLearnedModel(data):
	#Parse the learned portion of the model into a usable structure (np.array?)
	
	#Add the learned portion to the base prediction
	
	#Create a message for the full estimated map and add it to queue
	
	#Run the planning algorithm on the estimated map 
	
	#Build a waypoint command message to send to pixhawk
	
	#If in autonomous planning mode
		#Send the waypoint cmd to pixhawk
		
	#send the desired output waypoint command message to the GCS
	
	#end loop

#Build the Node Packet for Subscribers to let the network manager know what this process wants
def buildNodePacket(subscribers, PacketCounterNode): 
	pkt_id = PyPacket.PacketID(PyPacket.PacketPlatform.DUMMY,10)
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
		c += 1
	#End loop
	#serialize the data
	data_str = msg.SerializeToString()
	pkt.setData(data_str) #normally insert a data building part
	pkt.displayPacket()
	del msg
	return pkt.getPacket() #return the byte array msg


'''
To communicate sensor measurements and learned fields to the ground station for monitoring
Uses Network Manager Process
'''
class TelemetryTask(threading.Thread):

	def __init__(self)
		threading.Thread.__init__(self)
		
		#create logger
		
		#network setup
		self.PORT = 14120
		self.IP = 'localhost'
		self.ID = 'AC'
		self.Num = 10
		self.GCS_Num = 1
		
		#Generate a list of subscribers Here
		self.sublist = []
		#GCS COMMAND MSG
		self.sublist.add(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_GCS_CMD_MSG, PyPacket.PacketID(PyPacket.PacketPlatform.GROUND_CONTROL_STATION,self.GCS_Num).getBytes,self.PORT,'localhost') #GCS Command Msgs
		#RF LEARNED MODEL
		self.sublist.add(Subscriber.Subscriber(PyPacket.PacketDataType.PKT_RF_MODEL_MSG, PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,self.Num).getBytes,self.PORT,'localhost') #RF Model from GP Task
		
		
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
		
		#Loop Forever
		while not shutdown_event.is_set():
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
					parseCommand(newPkt.getData)
				elif isLearnedModel(newPkt):
					parseLearnedModel(newPkt.getData)
				else:
					#Unexpected message 
			#END 
			
			for s in writable:
				#Check to see if our output queue is empty
				try:
					next_msg = msg_queue.get_nowait()
				except Queue.Empty:
					#queue is empty or blocked
					time.sleep(0.01)
				else:
					s.sendto(next_msg[1],next_msg[0])
		#End While Loop
		
		my_out_socket.close()
		my_in_socket.close()
		
		print('\tTelemtryTask [closed]')
		
class SensingTask(threading.Thread):
	def __init__(self)
		#NEED TO IMPORT NODE GAINS
		threading.Thread.__init__(self)
		
		#important parameters
		self.centerLat = 40.2
		self.centerLon = -105.1
		self.refAlt = 0
		tx_power = 10 #dBm 
		tx_gains = 3 #dB
		rx_gain = 3 #dB
		
		
		#Load the propagation model into memory from file
		#np.array
		self.xgridResolution = 50
		self.ygridResolution = 50 
		
	def run(self)
		
		#Build the data type
		rf_data_msg = PyPackets_pb2.RF_Data_Msg()
		rf_data_msg.id = self.AIRCRAFT_ID
		
		#Build the packet
		#-TODO-
		newPacket = PyPacket.PyPacket()
		newPacket.setDataType(PyPacket.PacketDataType.PKT_RF_DATA_MSG)
		newPacket.setID(PyPacket.PacketID(PyPacket.PacketPlatform.AIRCRAFT,aircraft_id_num).getBytes())
		
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
					#How are we generating data: For now just randi
					rf_data_msg.lla.x = 40.1#lat
					rf_data_msg.lla.y = -105.3#lon
					rf_data_msg.lla.z = 110#alt
					
					#optional Skip
					
					#RF Part
					for s in range (0,2):
						#Each Node 
						new = rf_dta_msg.rfNode.add()
						new.chanID = str(s) #which node is it
						new.rssi = 0-random.uniform(60,100) #Eventually replaced with simulation model
						new.pl_msr = -(new.rssi - self.rx_gain - self.tx_gains - self.tx_power)
						new.pl_prediction_error  = 10 - new.pl_msr #Place holder
					#end loop
					
					#time
					rf_data_msg.time = time.time()
					rf_data_msg.packetNum = rf_msg_counter
					#increment counter
					rf_msg_counter = rf_msg_counter + 1
					
					#log message
					print('Beacon message number %i with time value: %f ' % (rf_data_msg.packetNum, rf_data_msg.time)
					#serialize 
					data_str = rf_data_msg.SerializeToString()
					
					#Put the data into a packet
					newPacket.setData(data_str)
					
					#Add the packet to the global send queue
					msg_queue.put(newPkt.getPacket())
					
				#-----------------
				#Normal Mode
				#-----------------
				elif self.SensorMode == 1:
					#Pull vehicle data using dronekit
					rf_data_msg. = vehicle.location.global_frame.lat
					rf_data_msg. = vehicle.location.global_frame.lon
					rf_data_msg. = vehicle.location.global_relative_frame.alt
					#Optional
					rf_data_msg. = vehicle.attitude.roll
					rf_data_msg. = vehicle.attitude.pitch
					rf_data_msg. = vehicle.attitude.yaw
					rf_data_msg. = vehicle.airspeed
					
					#-First determine indexes
					ENU = lla2flatENU(rf_data_msg.lla.x,rf_data_msg.lla.y rf_data_msg.lla.z,self.centerLat,self.centerLon,self.refAlt)
					xgridNum = ENU[0] / self.xgridResolution #NOT QUITE CORRECT
					ygridNum = ENU[1] / self.ygridResolution
					
					#-get the propagated value from indexes
					pl_predicted = self.propagated_PathLoss[xgridNum][ygridNum]

					#RF data part
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
						
						#log message
						print('Beacon message number %i with time value: %f ' % (rf_data_msg.packetNum, rf_data_msg.time)
						#serialize 
						data_str = rf_data_msg.SerializeToString()
						
						#Put the data into a packet
						newPacket.setData(data_str)
						
						#Add the packet to the global send queue
						msg_queue.put(newPkt.getPacket())
						
					except: 
						#log message
						print "serial error with sensors"
					#End try loop
					del rf_data_msg.rfNode[:]
				
				#-----------------
				#Unknown Mode
				#-----------------
				else:
					pass
				#END MODE LOOPS
			else:
				time.sleep(0.01)
			#END FREQ LOOPS
		#END WHILE
		self.sock.close()
		print('\t SensorTask [closed]')
			
	

if __name__  == "__main__":
	#Arguments
	#TBD
	
	#Parse Arguments; For now assume defaults
	parser = argparse.ArgumentParser(description='Comm-Aware Autonomous Application for aircraft')
	parser.add_argument('AIRCRAFT_ID',type=str)
	#Repeat as needed
	
	nodeA_latitude =
	nodeA_longitude = 
	
	nodeB_latitude =
	nodeB_longitude = 
	
	#NEEDED?
	GCSPosition = [LAT,LON] #turn into cLat,cLon
	
	#Geofence Boundary (ENU)
	xmin = -2000
	xmax = 2000
	ymin = -2000
	ymax = 2000 
	#DO WE NOT WANT NEGATIVES?
	
	#Start Telemetry Thread
	telem = TelemetryTask()
	telem.start()
	#Start Sensing Thread
	sensing = SensingTask()
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
	
	
	
	