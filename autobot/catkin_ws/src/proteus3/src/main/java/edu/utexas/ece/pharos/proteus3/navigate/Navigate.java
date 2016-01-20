package edu.utexas.ece.pharos.proteus3.navigate;

import edu.utexas.ece.pharos.logger.Logger;


/**
 * The parent class of all navigation components.
 * 
 * @author Chien-Liang Fok
 */
public abstract class Navigate {
	
	/**
	 * A constructor.
	 */
	public Navigate() {
	}

	/**
	 * Calculates the angle from currLoc to the targetLoc.  The return angle is from -PI to PI
	 * where 0 is due North and West is positive.
	 * 
	 * @param currLoc The current location
	 * @param targetLoc the target location
	 * @return the angle in radians of the vector from currLoc to targetLoc.
	 */
	public static double angle(Location currLoc, Location targetLoc) {
		
		// When a robot is facing north, its heading angle is zero, 
		// thus xerr is the difference in the latitudes, and
		// yerr is the difference in the longitudes.  The inverse of yerr is taken
		// because the more negative the longitude value, the more West it is,
		// and West is along the *positive* axis of the headings coordinate system.
		double xerr = targetLoc.latitude() - currLoc.latitude();
		double yerr = -1*(targetLoc.longitude() - currLoc.longitude());
		
		return Math.atan2(yerr, xerr);
	}
	
	/**
	 * Calculates the distance and change in heading required to get to the target destination.
	 * 
	 * @param currLoc The current location.
	 * @param currHeading The current heading in radians
	 * @param targetLoc The target location.
	 * @return A TargetDirection object containing the distance and change in heading necessary
	 * to get to the destination.
	 */
	public TargetDirection locateTarget(Location currLoc, double currHeading, Location targetLoc) {
		if (currLoc == null) {
			Logger.logErr("currLoc is null!");
			return null;
		}
		double distance =  currLoc.distanceTo(targetLoc);
		
		// When a robot is facing north, it's heading angle is zero, 
		// thus the xerr is the difference in the latitudes, and its
		// yerr is the difference in the longitudes.  The yerr is inverse
		// because the more negative the longitude value, the more West it is,
		// and West is along the *positive* axis of the headings coordinate system.
//		double xerr = targetLoc.latitude() - currLoc.latitude();
//		double yerr = -1*(targetLoc.longitude() - currLoc.longitude());
//		
//		double angleToTarget = Math.atan2(yerr, xerr);
		double angleToTarget = angle(currLoc, targetLoc);
		double headingErr = headingError(currHeading, angleToTarget);

		if (System.getProperty ("PharosMiddleware.debug") != null) {
			StringBuffer sb = new StringBuffer("locateTarget(): ");
			sb.append("Current State as of time " + System.currentTimeMillis() + ":\n\tLocation Data:\n");
			sb.append("\t\tCurrent Location: (" + currLoc.latitude() + ", " + currLoc.longitude() + ")\n");
			sb.append("\t\tTarget Location: (" + targetLoc.latitude() + ", " + targetLoc.longitude() + ")\n");
			sb.append("\t\tDistance: " + distance + " meters\n");
			sb.append("\tHeading Data:\n");
			sb.append("\t\tAngle to target: " + angleToTarget + " radians\n");
			sb.append("\t\tCurrent Heading: " + currHeading + " radians\n");
			sb.append("\t\tHeading Error: " + headingErr + " radians (" + ((headingErr < 0) ? "Must Turn Right!" : "Must Turn Left!") + ")");
			Logger.log(sb.toString());
		}
		
		return new TargetDirection(distance, headingErr);
	}
	
	/**
	 * Calculates the change in heading necessary to make the robot turn in the direction
	 * of the target.  Both of the input parameters must be in radian units in the range
	 * -PI to +PI.  The return value is also in radians in the range of -PI to +PI.
	 * A negative return value means the robot must turn right (clockwise when viewed from above)
	 * to face the target destination.  A positive return values means the robot must turn left
	 * (counter-clockwise when viewed from above) to face the target.
	 * 
	 * @param currHeading  The current direction in which the robot is moving in radian units between
	 * -PI and +PI.
	 * @param angleToTarget The direction in which the robot must move to head towards the target 
	 * in radian units between -PI and +PI.
	 * @return The change in heading angle necessary to make the robot turn towards the target.  A negative
	 * value indicates a right turn, and a positive value indicates a left turn.
	 */
	public static double headingError(double currHeading, double angleToTarget) {
		
		// First convert both heading measurements to be between 0 and 2*PI
		if (currHeading < 0) currHeading = 2*Math.PI + currHeading;
		if (angleToTarget < 0) angleToTarget = 2*Math.PI + angleToTarget;
		
		double headingErr = angleToTarget - currHeading;
		
		if (headingErr > Math.PI)
			headingErr = headingErr - 2*Math.PI;
		else if (headingErr < -1*Math.PI)
			headingErr = 2*Math.PI + headingErr;
		
		return headingErr;
	}
	
//	private static void log(String msg) {
//		String result = "Navigate:" + msg;
//		
//		// Only print text out if in debug mode
//		if (System.getProperty ("PharosMiddleware.debug") != null)
//			System.out.println(result);
//		
//		// Always print the log text to a file if the FileLogger exists
//		if (flogger != null)
//			flogger.log(result);
//	}
}