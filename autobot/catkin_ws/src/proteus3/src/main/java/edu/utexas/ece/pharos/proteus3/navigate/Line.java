package edu.utexas.ece.pharos.proteus3.navigate;

/**
 * A line connects two locations.  This class
 * provides various methods for analyzing locations 
 * on and around a line.
 * 
 * @author Chien-Liang Fok
 */
public class Line {
	double slope;
	double yIntersect;
	
	boolean isVertical = false;
	double xIntersect = 0;
	
	Location startLoc = null, stopLoc = null;
	
	public Line(Location startLoc, Location stopLoc) {
		
		this.startLoc = startLoc;
		this.stopLoc = stopLoc;
		
		if (startLoc.longitude() == stopLoc.longitude()) {
			isVertical = true;
			xIntersect = startLoc.longitude();
		} else  {
			slope = (startLoc.latitude() - stopLoc.latitude()) / (startLoc.longitude() - stopLoc.longitude());
			yIntersect = startLoc.latitude() - slope * startLoc.longitude();
		}
	}
	
	public Line(double slope, double yIntersect) {
		this.slope = slope;
		this.yIntersect = yIntersect;
	}
	
	public Location getStartLoc() {
		return startLoc;
	}
	
	public Location getStopLoc() {
		return stopLoc;
	}
	
	public double getLongitude(double latitude) {
		if (isVertical)
			return xIntersect;
		else
			return (latitude - yIntersect) / slope;
	}
	
	public double getLatitude(double longitude) {
		if (isVertical) {
			if (longitude == xIntersect)
				return Double.MAX_VALUE;
			else
				return Double.NaN;
		} else
			return slope * longitude + yIntersect;
	}
	
	/**
	 * Returns the location on the line that is the closest to the specified location.
	 * 
	 * @param loc The specified location
	 * @return The closest location
	 */
	public Location getLocationClosestTo(Location loc) {
		if (isVertical) {
			/*
			 * Since this line is vertical, the shortest distance is the
			 * difference in longitudes (the latitude can be anything)
			 */
			Location closestLoc = new Location(loc.latitude(), xIntersect);
//			System.out.println("Calculating shortest distance between line: " + toString() + " and " + loc);
			return closestLoc;
		} else if (slope == 0) {
			/*
			 * Since this line is horizontal, the shortest distance is the
			 * difference in latitudes (the longitude does not matter)
			 */
			Location closestLoc = new Location(yIntersect, loc.longitude());
			return closestLoc;
		} else {
			Line perpendicularLine = getPerpendicularLine(loc);
//			System.out.println("Perpendicular line: " + perpLine);
			Location intersectionPoint = getIntersection(perpendicularLine);
//			System.out.println("Intersection point: " + intersectionPoint);
//			System.out.println("Target Point: " + loc);
			return intersectionPoint;
		}
//		if (Double.isNaN(result)){
//			System.err.println("WARNING: Line.shortestDistanceTo(Location) calculated NaN:");
//			System.err.println("\tLine: " + toString());
//			System.err.println("\tLocation: " + loc);
//			System.exit(-1);
//		}
//		
//		return result;
	}
	
	/**
	 * 
	 * Returns the location that the robot would have been at if it had traveled along
	 * this line for the specified distance.  It finds this location using a divide-and-conquer search.
	 *
	 * @param distanceTraveled The distance in meters that the robot
	 * traveled from the startLoc towards the destLoc.
	 * @return The closest location.
	 */
	public Location getLocationRelativeSpeed(double distanceTraveled) {
		
		/*
		 * This is the location along this line that the robot would
		 * have been if it had traveled the specified distance from
		 * the startLoc to the destLoc.
		 */
		//Location idealLoc = null;
		
		/*
		 * Iteratively search for the location that is "distanceTraveled" away from
		 * the startLoc heading towards the stopLoc along this line.
		 */
		Location startLoc = this.startLoc;
		Location stopLoc = this.stopLoc;
		Location midPoint = getMidPoint(startLoc, stopLoc);
		double currDist = midPoint.distanceTo(this.startLoc);
		
		// some debug output
//		System.out.println("Line.shortestDistanceTo(loc, distanceTraveled): loc=" + loc + ", distanceTraveled=" + distanceTraveled);
//		
//		System.out.println("\tstarLoc = " + startLoc);
//		System.out.println("\tstopLoc = " + stopLoc);
//		System.out.println("\tmidPoint = " + midPoint);
//		System.out.println("\tcurrDist = " + currDist + ", distanceTraveled = " + distanceTraveled + ", diff = " + (currDist - distanceTraveled));
		
//		int round = 0;
		
		double prevCurrDist = 0;
		
		while (Math.abs(currDist - distanceTraveled) > 0.01 /* the correctness threshold */
				&& prevCurrDist != currDist /* prevent infinite loops when double precision is insufficient*/) 
		{
			if (currDist > distanceTraveled) {
				/*
				 * The midpoint is too far away, search the half that is closer to the 
				 * start Loc.
				 */
				stopLoc = midPoint;
			} else {
				/*
				 * The midpoint is too close, search the half that is closer to the 
				 * end Loc.
				 */
				startLoc = midPoint;
			}
			midPoint = getMidPoint(startLoc, stopLoc);
			prevCurrDist = currDist;
			currDist = midPoint.distanceTo(this.startLoc);
			
//			System.out.println("Iteration: " + round++);
//			System.out.println("\tstarLoc = " + startLoc);
//			System.out.println("\tstopLoc = " + stopLoc);
//			System.out.println("\tmidPoint = " + midPoint);
//			System.out.println("\tcurrDist = " + currDist + ", distanceTraveled = " + distanceTraveled + ", diff = " + (currDist - distanceTraveled));
		}
//		System.out.println("Ideal point: " + midPoint);
		return midPoint;
	}
	
	/**
	 * Calculates the mid-point between startLoc and stopLoc.
	 * 
	 */
	private Location getMidPoint(Location startLoc, Location stopLoc) {
		double avgLat = (startLoc.latitude() + stopLoc.latitude()) / 2;
		double avgLon = (startLoc.longitude() + stopLoc.longitude()) / 2;
		return new Location(avgLat, avgLon);
	}
	
	/**
	 * Returns the line that is perpendicular to this one that passes through
	 * the specified location.
	 * 
	 * @param loc The location through which the perpendicular line should pass.
	 * @return The perpendicular line.
	 */
	public Line getPerpendicularLine(Location loc) {
		// The perpendicular line has an inverse slope of this line
		double pSlope = -1 * (1/slope);
		double pYIntersect = loc.latitude() - pSlope * loc.longitude();
		return new Line(pSlope, pYIntersect);
	}
	
	/**
	 * Returns the point of intersection between this line and the
	 * specified line.
	 * 
	 * @param l The specified line.
	 * @return The point of intersection.
	 */
	public Location getIntersection(Line l) {
		double longitude = (l.yIntersect - yIntersect) / (slope - l.slope);
		double latitude = getLatitude(longitude);
		
		Location result = new Location(latitude, longitude);
		
		// Some debug statements...
//		System.out.println("Calculating intersection of: " );
//		System.out.println("\tLine 1: " + toString());
//		System.out.println("\tLine 2: " + l.toString());
//		System.out.println("\tIntersection: " + result);
//		
//		System.out.println("\tgetLatitude(result.longitude()): " + getLatitude(result.longitude()));
//		System.out.println("\tgetLongitude(result.latitude()): " + getLongitude(result.latitude()));
//		
//		System.out.println("\tl.getLatitude(result.longitude()): " + l.getLatitude(result.longitude()));
//		System.out.println("\tl.getLongitude(result.latitude()): " + l.getLongitude(result.latitude()));
//		
		return result;
	}
	
	/**
	 * Determines whether the specified location is left of or right of the specified line.
	 * 
	 * @param loc The location.
	 * @param line The line.
	 * @return true if the location is left of the line.
	 */
	public static boolean isLocationLeftOf(Location loc, Line line) {
		
//		Logger.logDbg("\n------------------\nBegin method call:\n\tLocation: " + loc + "\n\tLine: " + line + "\n\tLine start: " + line.getStartLoc() + "\n\tLine stop: " + line.getStopLoc());
		Location lineStart = line.getStartLoc();
		
		// Calculate the line's heading.  This is a value between -PI to PI.
		double lineHeading = edu.utexas.ece.pharos.proteus3.navigate.Navigate.angle(lineStart, line.getStopLoc());
//		Logger.logDbg("Heading of line: " + lineHeading);
		
		// Calculate the heading to the location.  This is a value between -PI to PI.
		double headingToLoc = edu.utexas.ece.pharos.proteus3.navigate.Navigate.angle(lineStart, loc);
//		Logger.logDbg("Heading to target location: " + lineHeading);
		
		// Rotate the axis such that the heading of the line is due north (0 radians)
		double rotatedHeadingToLoc = headingToLoc - lineHeading;
//		Logger.logDbg("Rotated heading to target location: " + rotatedHeadingToLoc);
		
		// Ensure the rotated heading to loc remains between -PI to PI
		if (rotatedHeadingToLoc < -Math.PI)
			rotatedHeadingToLoc = Math.PI - rotatedHeadingToLoc;
		else if (rotatedHeadingToLoc > Math.PI)
			rotatedHeadingToLoc = rotatedHeadingToLoc - 2 * Math.PI;
//		Logger.logDbg("Rotated heading to target location between -PI to PI: " + rotatedHeadingToLoc);
		
		if (rotatedHeadingToLoc > 0) {
//			Logger.logDbg("location is left of the line!\n------------------\n");
			return true;
		} else {
//			Logger.logDbg("location is right of the line!\n------------------\n");
			return false;
		}
	}
	
	public String toString() {
		if (isVertical)
			return "vertical, longitude = " + xIntersect;
		else
			return "latitude = " + slope + " * longitude + " + yIntersect;
	}
}