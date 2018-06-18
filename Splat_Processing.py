#SPLAT! Data processing 

#IMPORTS
import os
import sys
import math
import struct
sys.path.insert(0,'../PyUAS') #get the path to the PyUAS Folder
sys.path.insert(0,'../PyUAS/protobuf') #get the path to the protobuf format
import assorted_lib

path_to_splat = ""

#Method 1: Take into account the elevation to maintain a constant ASL value for the propagation prediction

#Method 2: Assume negligible terrain elevation changes. AGL is constant 

#Define RoI (Center Lat, Center Lon, grid spacing)  #arguments or file

#Define antenna locations (Assume stationary for now) and their properties

#loop through each antenna setup and generate the splat! data 
	#call cmd line splat!
	#load in the splat! data 
	#(See matlab code for htis section)
	#Loop through and limit into RoI by Lat/Lon max-min 
	#assign values for each of the grids 

	#Save file and output in new binary form (header + row/column of floats for the PL value)
def SPLATDataGenerationMethod2(Aircraft_Height):
	RoIFilename = "RoI_Antennas.txt"
	#load in RoI and Antenna info from file
	meta_data_f = open(RoIFilename,'r')
	#Skip the defintions
	for a in xrange(0,9):
		print (meta_data_f.readline()) #for checking purposes
	#Read the RoI info
	centerLat = float(meta_data_f.readline()) #cast as float
	centerLon = float(meta_data_f.readline()) #cast as float
	north = float(meta_data_f.readline()) 
	south = float(meta_data_f.readline())
	east = float(meta_data_f.readline())
	west = float(meta_data_f.readline())
	xspacing = float(meta_data_f.readline())
	yspacing = float(meta_data_f.readline())
	#Use the max position corners 
	
	LLA = assorted_lib.ENU2LLA([-west,-south,0],[centerLat,centerLon])
	#Max Lat/Lon 
	maxLat = LLA[0]
	maxLon = LLA[1]
	
	LLA = assorted_lib.ENU2LLA([-west,-south,0],[centerLat,centerLon])
	#min Lat/Lon
	minLat = LLA[0]
	minLon = LLA[1]
	
	xgridlength = (east + west)/xspacing
	ygridlength = (north + south)/yspacing
	#center index
	xcenterIndex = math.ceil(xgridlength/2);
	ycenterIndex = math.ceil(ygridlength/2);
	
	#Path Loss field
	PathLossField = {}
	#Assign all the initial values to be 0
	c = 0
	for a in xrange(0,int(xgridlength)):
		for b in xrange(0,int(ygridlength)):
			PathLossField[a,b] = 0
			#PathLossField[a,b] = c
			#c += 1
	
	
	Antenna = []
	for line in meta_data_f:
		print(line)
		Antenna.append(line)
	
	#Now split the antenna
	r = 10
	strvalues = []
	for ant in Antenna:
		print ant
		strvalues = ant.split(",")
		name = strvalues[0]
		lat = float(strvalues[1])
		lon = -float(strvalues[2]) #invert the longitude
		height = float(strvalues[3])
		pt = float(strvalues[4])
		gt = float(strvalues[5])
		#Write out the Transmitter file
		transmitterfilename = name + ".qth"
		#what is the path to SPLAT!?
		filename = path_to_splat + transmitterfilename
		print filename
		tf = open(filename,'w')
		tf.write(name + '\n')
		tf.write(str(lat) + '\n')
		tf.write(str(lon) + '\n')
		tf.write(str(height) + " meters")
		tf.close()
		
		outfile = name + "_" + str(Aircraft_Height) + "_output.dat"
		print outfile
		#Run the splat cmd with the transmitter file
		print 'splat-hd -t ' + filename + ' -L ' + str(Aircraft_Height) + ' -R ' + str(r) + ' -ano ' + outfile
		os.system('splat-hd -t ' + filename + ' -L ' + str(Aircraft_Height) + ' -R ' + str(r) + ' -ano ' + outfile)
		
		#read in the data
		splatfile = open(outfile,'r')
		
		#ignore the first two lines
		splatfile.readline() 
		splatfile.readline()
		#Read in each line
		datapoints = []
		for line in splatfile:
			datapoints.append(line)
		splatfile.close()
		#Now process each point
		values = []
		for point in datapoints:
			values = point.split(",")
			thisLat = float(values[0])
			thisLon = -float(values[1]) #invert this longitude
			#compare with max/min values for latitude and longitude
			if(thisLat < maxLat and thisLat > minLat and thisLon < maxLon and thisLon > minLon):
				#get the rest of the values
				pl = float(values[4])
				#get the x/y value
				ENU = LLA2ENU([thisLat,thisLon,0],[centerLat,centerLon])
				#find the correct grid
				#-------X----------
				xgrid = findGridIndex(ENU[0],xspacing,xcenterIndex,xgridlength)
				#-------Y----------
				ygrid = findGridIndex(ENU[1],yspacing,ycenterIndex,ygridlength)
				#add the pl value to the correct grid
				#PathLossField[xgrid,ygrid] = PathLossField[xgrid,ygrid] + pl
			#else just end
		#End loop
		thisoutfile = name + "_data_" + str(Aircraft_Height) + ".sdg"
		#We have now processed all teh data for a file; time to generate the binary data file for 
		datafile = open(thisoutfile,"wb")
		#create a single string
		#outstring = str(centerLat) + str(centerLon) + str(east) + str(west) + str(xspacing) + str(north) + str(south) + str(yspacing)
		datafile.write(struct.pack('8f',centerLat,centerLon,east,west,xspacing,north,south,yspacing))
		for b in range(0,int(ygridlength)):
			for a in range(0,int(xgridlength)): 
				#outstring += str(PathLossField[a,b])
				datafile.write(struct.pack('f',PathLossField[a,b])) #write the float to the file
				#print("wrote this value",PathLossField[a,b])
		#write that string out
		#datafile.write(bytearray(outstring))
		datafile.close()
	#End of antenna loop
	
	#end of function
	
#Find Indexes given a LAT LON
def findGridIndex(V, spacing, centerInd, vlength):
	offset_index = V/spacing
	new_index = centerInd + round(offset_index)
	if (new_index < 0):
		return 0
	else:
		out = int(round(new_index))
		if (out > vlength):
			return int(vlength)
		else:
			return out

def testGridIndexSystem():
	centerLat = 40.145177
	centerLon = -105.243515
	north = 2000 
	south = 2000
	east = 2000
	west = 2000
	xspacing = 25
	yspacing = 25
	#Use the max position corners 
	
	LLA = assorted_lib.ENU2LLA([-west,-south,0],[centerLat,centerLon])
	#Max Lat/Lon 
	maxLat = LLA[0]
	print("Max Latitude is %f" % maxLat)
	maxLon = LLA[1]
	print("Max Longitude is %f" % maxLon)
	LLA = assorted_lib.ENU2LLA([-west,-south,0],[centerLat,centerLon])
	#min Lat/Lon
	minLat = LLA[0]
	print("Min Latitude is %f" % minLat)
	minLon = LLA[1]
	print("Min Longitude is %f" % minLon)
	
	xgridlength = (east + west)/xspacing
	ygridlength = (north + south)/yspacing
	print("Grid: %f, %f" % (xgridlength,ygridlength))
	#center index
	xcenterIndex = math.ceil(xgridlength/2)
	ycenterIndex = math.ceil(ygridlength/2)
	print("Center Indexes: %i, %i" %(xcenterIndex,ycenterIndex))
	 
	for x in range(-2000,2000):
		grid = findGridIndex(x,xspacing,xcenterIndex,xgridlength)
		print("Grid %i for position %i" % (grid, x))
	
	
def findPLinFile(a,b,length):
	byteNumber = (length*b + a)*4 + 8*4
	return byteNumber

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

def testFile():
	filename = "NodeA_data_100.sdg"
	
	centerLat = 40.145177
	centerLon = -105.243515
	north = 1500 
	south = 1500
	east = 1500
	west = 1500
	xspacing = 50
	yspacing = 50
	
	xgridlength = (east + west)/xspacing
	ygridlength = (north + south)/yspacing
	xcenterIndex = math.ceil(xgridlength/2)
	ycenterIndex = math.ceil(ygridlength/2)
	
	dtest = {}
	f = open(filename,'r')
	for b in xrange(0,int(ygridlength)):
		for a in xrange(0,int(xgridlength)):
			byteNumber = (xgridlength*b + a)*4 + 8*4
			f.seek(byteNumber)
			print('Seeking to ', byteNumber)
			bytesIn = f.read(4)
			thisfloat = struct.unpack('f',bytesIn)
			print('The value is ', thisfloat[0])
			dtest[a,b] = thisfloat[0]
	
	for a in xrange(0,int(xgridlength)):
		for b in xrange(0,int(ygridlength)):
			print dtest[a,b]
	
if __name__  == "__main__":
	print('Starting Function')
	SPLATDataGenerationMethod2(100)
	#testGridIndexSystem()
	#testFile()
	print('Function Finished')