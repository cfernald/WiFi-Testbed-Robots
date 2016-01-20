#  GPS (UTC) time in seconds and microseconds since the epoch 
uint32 time_sec

# GPS (UTC) time in seconds and microseconds since the epoch 
uint32 time_usec

# Latitude in degrees
float32 latitude

# Longitude in degrees
float32 longitude

# Altitude in meters 
float32 altitude

# UTM WGS84 coordinates, easting [m] 
float64 utm_e

# UTM WGS84 coordinates, northing [m]
float64 utm_n

# Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix, 6 = Dead Reckoning 
uint8 quality

# Number of satellites in view
uint8 num_sats

# Horizontal dilution of position (HDOP)
float32 hdop

# Vertical dilution of position (VDOP)
float32 vdop

# Tracking angle of the GPS
float32 heading

# Horizontal error in meters
# float64 err_horz

# Vertical error in meters
# float64 err_vert
