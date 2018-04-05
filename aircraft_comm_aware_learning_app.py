


#Imports


def handleRFMsg(packetData):
	#we have just received a packets data
	datastr = str(packetData)
	#assign it to the google protobuf type
	rf_msg = PyPackets_pb2.RF_Data_Msg()
	rf_msg.ParseFromString(datastr)
	#get the relevant information out of the data structure
	#- position and errors in pathloss measurements
	#- add these to a global dictionary 
	
	

class TelemetryTask(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		
		#stuff
		
	def run():
		#iniitalize run conditionals
		
		def run(self):
		#Create Socket Objects
		my_out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		my_in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		
		my_in_socket.bind(('',16000))
		
		#Create lists for select
		in_sockets = [my_in_socket]
		out_sockets = [my_out_socket]
		
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
				if isRFMsg(newPkt):
					handleRFMsg(newPkt.getData)
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

class GaussianProcessTask(threading.Thread):
	def __init__(self,Logmode):
		threading.Thread.__init__(self)
		
		#Logger
		self.logger = logging.getLogger("CommAwareLearning:GaussianProcessTask")
		self.logger.setLevel(Logmode)
		
		#Initialize GP information (Check out my gp task from earlier versions)
		self.Kernel = Matern(length_scale = 25, nu = 5/2) + ConstantKernel() + WhiteKernel(noise_level=1)
		self.threshold = Threshold
		self.MAX_RANGE = MAX_RANGE
	
	def run():
		#Check out the gp from the subscriber 
		
		#Set the gaussian process
		gp = gaussian_process.GaussianProcessRegressor(kernel = self.Kernel)
		
		#Message type setup
		#_TODO_
		
		#Prediction Zone
		xpred = []
		grid_x_points = np.arange(xmin,xmax+1,xspacing)
		grid_y_points = np.arange(ymin,ymax+1,yspacing)
		
		for a in range(0,len(grid_x_points)):
			for b in range(0,len(grid_y_points)):
				xpred.append([grid_x_points[a],grid_y_points[b]])
		
		#transform the list to an array
		Xprediction = np.array(xpred)
		
		GPTimeLast = time.time()
		GPTimeNow = GPTimeLast()
		GPRate = 
		packetNum = 
		
		#Loop
		while not shutdown_event.is_set():
			if time.time() - GPTimeLast >= GPRate:
				#Get the data 
				GPS_RSSI_DATA_BUFFER = GPS_RSSI_DATA #??
				#Grab teh data
				Xdata = []
				Ydata = []
				for d in GPS_RSSI_DATA_BUFFER:
					sum1 = GPS_RSSI_DATA_BUFFER[d].get('sum')
					count1 = GPS_RSSI_DATA_BUFFER[d].get('count')
					currentAve = sum1/float(count1)
					x1 = GPS_RSSI_DATA_BUFFER[d].get('x_point')
					y1 = GPS_RSSI_DATA_BUFFER[d].get('y_point')
					Xdata.append([x1,y1])
					Ydata.append([currentAve])
				#Finished getting data; set the dtype
				X = np.array(Xdata, dtype = np.float32)
				Y = np.array(Ydata, dtype = np.float32)
				
				#Learn the Hyper Parameters
				self.logger.info("Learning the GP Hyper Parameters")
				gpStart = time.time()
				gp.fit(X,Y)
				gpFinished = time.time()
				self.logger.inf("Time Taken for GP Learning: %f" % (gpFinished-gpStart))
				
				#Predicting with learned model
				self.logger.info("Predicting with Learned GP Model")
				gpStart = time.time()
				y_pred, sigma = gp.predict(xpred,return_std=True)
				gpFinished = time.time()
				self.logger.info("TimeTaken for GP Prediction: %f" % (gpFinished - gpStart))
				
				#Convert to mesh with Predicted value for a given X,Y Coord
				Z = np.zeros((len(grid_x_points),len(grid_y_points)))
				counter = 0
				 
				for a in range(0,len(grid_x_points)):
					for b in range(0,len(grid_y_points)):
						Z[a,b] = y_pred[counter]
						counter = counter + 1
		
		
		
	