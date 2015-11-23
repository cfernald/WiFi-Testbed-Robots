package edu.utexas.ece.pharos.proteus3.navigate;

public class Location implements java.io.Serializable, Comparable<Location> {
	private static final long serialVersionUID = -2689555631414682934L;
	private double latitude, longitude, elevation;
	
	/**
	 * A constructor that extract location information from GPS sensor data.
	 * 
	 * @param gpsData The GPS sensor data from the Player middleware.
	 */
	//public Location(PlayerGpsData gpsData) {
//			this(gpsData.getLatitude()/1e7, gpsData.getLongitude()/1e7, gpsData.getAltitude());
//	}
	
	/**
	 * A constructor that takes the latitude and longitude.  
         * The elevation is assumed to be zero.
	 * 
	 * @param latitude The latitude.
	 * @param longitude The longitude.
	 */
	public Location(double latitude, double longitude) {
		this(latitude, longitude, 0);
	}
	
	/**
	 * A constructor that takes the latitude, longitude, and elevation explicitly.
	 * 
	 * @param latitude The latitude.
	 * @param longitude The longitude.
	 * @param elevation The elevation.
	 */	
	public Location(double latitude, double longitude, double elevation) {
		this.latitude = latitude;
		this.longitude = longitude;
		this.elevation = elevation;
	}
	
	public boolean equals(Object obj) {
		if (obj != null) {
			if (obj instanceof Location) {
				Location coord = (Location)obj;
				return coord.latitude() == latitude() && 
					coord.longitude() == longitude() &&
					coord.elevation() == elevation();
			}
		}
		return false;
	}
	
	public double latitude() {
		return latitude;
	}
	
	public double longitude() {
		return longitude;
	}
	
	public double elevation() {
		return elevation;
	}
	
	@Override
	public int hashCode() {
		return (int)(latitude * 100000 + longitude * 100000);
	}
	
	/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
	/*::                                                                         :*/
	/*::  This routine calculates the distance between two points (given the     :*/
	/*::  latitude/longitude of those points). It is being used to calculate     :*/
	/*::  the distance between two ZIP Codes or Postal Codes using our           :*/
	/*::  ZIPCodeWorld(TM) and PostalCodeWorld(TM) products.                     :*/
	/*::                                                                         :*/
	/*::  Definitions:                                                           :*/
	/*::    South latitudes are negative, east longitudes are positive           :*/
	/*::                                                                         :*/
	/*::  Passed to function:                                                    :*/
	/*::    lat1, lon1 = Latitude and Longitude of point 1 (in decimal degrees)  :*/
	/*::    lat2, lon2 = Latitude and Longitude of point 2 (in decimal degrees)  :*/
	/*::    unit = the unit you desire for results                               :*/
	/*::           where: 'M' is statute miles                                   :*/
	/*::                  'K' is kilometers (default)                            :*/
	/*::                  'N' is nautical miles                                  :*/
	/*::  United States ZIP Code/ Canadian Postal Code databases with latitude & :*/
	/*::  longitude are available at http://www.zipcodeworld.com                 :*/
	/*::                                                                         :*/
	/*::  For enquiries, please contact sales@zipcodeworld.com                   :*/
	/*::                                                                         :*/
	/*::  Official Web site: http://www.zipcodeworld.com                         :*/
	/*::                                                                         :*/
	/*::  Hexa Software Development Center (c) All Rights Reserved 2004          :*/
	/*::                                                                         :*/
	/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
	private static double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
		
		// check for special condition of exact match
		//if ((lat1 - lat2) < 0.000001 && (lon1 - lon2) < 0.000001) return 0;
		if (lat1 == lat2 && lon1 == lon2) return 0;
		
		double theta = lon1 - lon2;
		
		double dist = Math.sin(deg2rad(lat1)) * Math.sin(deg2rad(lat2)) + Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) * Math.cos(deg2rad(theta));
		
//		System.out.println("theta=" + theta + ", dist = " + dist);
		
		// HACK:  not sure why this would ever happen
		// TODO: Remove this after figuring out why this happens. See Mission 9, experiment 3, Relative Divergence calculation for an example of when it happens
		if (dist > 1) dist = 1;
		
		dist = Math.acos(dist);
//		System.out.println("After acos(...), dist = " + dist);
		
		dist = rad2deg(dist);
//		System.out.println("After rad2deg(...), dist = " + dist);
		
		dist = dist * 60 * 1.1515;
		if (unit == 'K') {
			dist = dist * 1.609344;
		} else if (unit == 'N') {
			dist = dist * 0.8684;
		}
		return (dist);
	}

	/**
	 * This function converts decimal degrees to radians
	 * 
	 * @param deg angle measurement in decimal degree units
	 * @return angle measurement in radians
	 */
	private static double deg2rad(double deg) {
		return (deg * Math.PI / 180.0);
	}

	/**
	 * This function converts radians to decimal degrees   
	 * 
	 * @param rad angle measurement in radian units
	 * @return angle measurement in degrees
	 */
	private static double rad2deg(double rad) {
		return (rad * 180.0 / Math.PI);
	}

	/**
	 * Returns the distance between two GPS coordinates in meters.
	 * 
	 * @param targetLoc The destination location
	 * @return The distance between currLoc and targetLoc in meters.
	 */
	public double distanceTo(Location targetLoc) {
		double result = distance(latitude(), longitude(), 
				targetLoc.latitude(), targetLoc.longitude(), 'K');
		return result * 1000; // convert to meters
	}
	
	@Override
	public int compareTo(Location o) {
		if (o instanceof Location) {
			Location l = (Location)o;
			if (l.latitude() < latitude())  // locations that are more North are first
			return 1;
		else if (l.latitude() > latitude())
			return -1;
		else {
			// The latitudes are equal
			if (l.longitude() > longitude())  // locations that are more East are first
				return -1;
			else if (l.longitude() < longitude()) 
				return 1;
			else
				return 0; // they are equal
		}
//			if (l.longitude() < longitude())  // locations that are more East are first
//				return 1;
//			else if (l.longitude() > longitude())
//				return -1;
//			else {
//				// The longitudes are equal
//				if (l.latitude() > latitude())  // locations that are more North are first
//					return -1;
//				else if (l.latitude() < latitude()) 
//					return 1;
//				else
//					return 0; // they are equal
//			}
		} else
			return 0;
	}
	
	public String toString() {
		return "(" + latitude + ", " + longitude + ", " + elevation + ")";
	}
}

