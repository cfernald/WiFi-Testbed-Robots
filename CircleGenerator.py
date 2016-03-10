import sys, math
def genCircle():
	f = open('circle.csv','w')
	radiusOfEarth = 6371.0
	start_lat =float( sys.argv[1])
	start_lon = float(sys.argv[2])
	radius = float(sys.argv[3])
	start_lat = math.radians(start_lat)
	start_lon = math.radians(start_lon)
	for i in range(0,361):
		heading = math.radians(i)	
		destLat = math.asin(math.sin(start_lat)*math.cos(radius/radiusOfEarth) + math.cos(start_lat)*math.sin(radius/radiusOfEarth)*math.cos(heading))
		destLon = start_lon + math.atan2(math.sin(heading)*math.sin(radius/radiusOfEarth)*math.cos(start_lat), math.cos(radius/radiusOfEarth) - math.sin(start_lat)*math.sin(destLat))
		destLat = math.degrees(destLat);
		destLon = math.degrees(destLon);
		destLon = (destLon+540.0)%360.0-180.0;	
		output = str(destLat) + ', ' + str(destLon) + '\n'
		f.write(output)

if __name__ == "__main__":
    genCircle()
