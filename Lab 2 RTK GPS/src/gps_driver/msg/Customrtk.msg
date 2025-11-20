std_msgs/Header header

#(latitude, longitude, height)
float64 latitude
float64 longitude
float64 altitude

#utm coordinates
float64 utm_easting
float64 utm_northing
uint8   zone
string  letter

#for checking quality
int32   fix_quality     #type: 0 = invalid, 1 = GPS fix, 4 = RTK fixed, 5 = RTK float
float64 hdop           

#raw nmea sentence for debugging
string  gngga_read
