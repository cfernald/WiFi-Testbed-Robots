package edu.utexas.ece.pharos.proteus3.navigate;

import proteus3_compass.CompassMsg;
import proteus3_gps.GPSMsg;

import edu.utexas.ece.pharos.exceptions.NoNewDataException;
import edu.utexas.ece.pharos.exceptions.NoValidDataException;
import edu.utexas.ece.pharos.logger.Logger;
import edu.utexas.ece.pharos.proteus3.sensors.CompassBuffer;
import edu.utexas.ece.pharos.proteus3.sensors.GPSBuffer;
import edu.utexas.ece.pharos.proteus3.mobilityPlanes.MobilityPlane;
import edu.utexas.ece.pharos.util.ThreadControl;

/**
 * Navigates a robot to a specified location using compass and GPS measurements.  
 * It operates in a cycle calculating the desired steering angle and speed and submiting
 * these commands to the mobility plane.
 * 
 * <p>This class depends on the compass and GPS sensors.  If either of these sensors fail to provide 
 * current data, it stops the robot.</p>
 * 
 * @author Chien-Liang Fok
 */
public class NavigateCompassGPS extends Navigate {
	/**
	 * The maximum acceptable age of a GPS measurement in milliseconds.
	 */
	public static final long MAX_GPS_AGE = 2500;
	
	/**
	 * The maximum acceptable age of a compass measurement in milliseconds.
	 */
	public static final long MAX_COMPASS_AGE = 500;
	
	/**
	 * Specifies the navigation component's cycle time in milliseconds.
	 * For example, a value of 100ms means the navigation process updates
	 * the direction and speed of the robot at 10Hz.
	 */
	public static final int NAV_CYCLE_PERIOD = 200;

	/**
	 * The threshold steering angle error in radians above which the robot will use
	 * a separate controller to turn towards the destination.
	 */
	public static final double MAJOR_HEADING_CORRECTION_THRESHOLD = 0.1744; // 10 degrees
	//public static final double MAJOR_HEADING_CORRECTION_THRESHOLD = 0.35; // 20 degrees

	/**
	 * The multiplier used when correcting for spatial errors.
	 * It was observed that the spatial error is typically between 0 and 1.5 meters.
	 */
	public static final double SPATIAL_ERROR_FIX = 0.1744; // 10 degrees

	/**
	 * The proportional gain constant of the navigation PID controller.
	 * This value controls the significance of the *present* error.
	 */
	public static final double Kp = 0.5;

	/**
	 * The integral gain constant of the navigation PID controller.
	 * This value controls the significance of the *past* error.
	 */
	public static final double Ki = 0.02;

	/**
	 * The derivative gain constant of the navigation PID controller.
	 * This value controls the significance of the *future* error.
	 */
	public static final double Kd = 0;

	private MobilityPlane mobilityPlane;

	private CompassBuffer compassBuffer;

	private GPSBuffer gpsBuffer;

	/**
	 * The total error of the system.
	 */
	private double totalError;

	/**
	 * The previous heading error.  This is used to calculate the change in error
	 * over time.
	 */
	private double previousError;

	/**
	 * The maximum turn angle in radians.  The value 0.35 radians (20 degrees) is for 
	 * the Traxxas mobility plane.
	 * 
	 * See: http://pharos.ece.utexas.edu/wiki/index.php/Proteus_III_Robot#Steering_Angle_Range
	 * TODO: This should be obtailed from the mobilityPlane.
	 */
	private double maxSteeringAngle = 0.35; 

	/**
	 * The speed of the robot (in m/s) while it is performing major corrections to its heading.
	 */
	private double headingCorrectionSpeed = 0.5;

	/**
	 * The threshold distance (in meters) to a waypoint before the robot
	 * concludes that it's "close enough" to the waypoint and stops 
	 * navigating.
	 */
	//public static final double GPS_TARGET_RADIUS_METERS = 2.5;
	public static final double GPS_TARGET_RADIUS_METERS = 4;

	/**
	 * Whether we are done navigating to a particular location.
	 */
	private boolean done;

	/**
	 * A constructor.
	 * 
	 * @param mobilityPlane The mobility plane being controlled.
	 * @param compassBuffer The compass data source (buffered).
	 * @param gpsBuffer The GPS data source (buffered).
	 */
	public NavigateCompassGPS(MobilityPlane mobilityPlane, CompassBuffer compassBuffer, 
			GPSBuffer gpsBuffer) 
	{
		this.mobilityPlane = mobilityPlane;
		this.compassBuffer = compassBuffer;
		this.gpsBuffer = gpsBuffer;
	}
	
	/**
	 * Navigates the robot to a particular location at a certain speed.
	 * 
	 * @param startLoc The ideal starting location.  If this is null, the current 
	 * location of the robot when this method is first called will be used.
	 * @param endLoc The destination location.
	 * @param speed The speed in meters per second at which the robot should travel.
	 * @return true if the robot successfully reached the destination
	 */
	public synchronized boolean go(Location startLoc, Location endLoc, double speed) {
		Logger.log("Method called: startLoc = " + startLoc + ", endLoc = " + endLoc + ", speed = " + speed);
		done = false;
		boolean success = false;

		int numTries = 0;
		while (startLoc == null) {
			Logger.logDbg("Starting location not specified, obtaining and using current location as start position.");
			try {
				numTries++;
				GPSMsg gpsMsg = gpsBuffer.getLocation(MAX_GPS_AGE);
				startLoc = new Location(gpsMsg.getLatitude(), gpsMsg.getLongitude());
				Logger.logDbg("Starting location set to " + startLoc);
			} catch(NoNewDataException nnde) {
				Logger.logErr("Stale location data (message: " + nnde.getMessage() + "), retrying in 1s (numTries = " + numTries + ")...");
				ThreadControl.pause(this, 1000);
			} catch(NoValidDataException nvde) {
				Logger.logErr("No valid location data, retrying in 1s (numTries = " + numTries + ")...");
				ThreadControl.pause(this, 1000);
			}
		}

		Line idealRoute = new Line(startLoc, endLoc);

		while (!done) {
			Location currLoc = null;

			// Get the current location...
			try {
				GPSMsg gpsMsg = gpsBuffer.getLocation(MAX_GPS_AGE);
				currLoc = new Location(gpsMsg.getLatitude(), gpsMsg.getLongitude());
			} catch(NoNewDataException nnde) {
				nnde.printStackTrace();
				Logger.logErr("Stale location data (message: " + nnde.getMessage() + "), halting robot...");	
				mobilityPlane.stop();
			} catch(NoValidDataException nvde) {
				nvde.printStackTrace();
				Logger.logErr("No valid location data, halting robot...");
				mobilityPlane.stop();
			}

			if (currLoc != null) {
				// Get the current heading ...
				try {
					CompassMsg compassMsg = compassBuffer.getHeading(MAX_COMPASS_AGE);
					done = doNextMotionTask(currLoc, Math.toRadians(compassMsg.getHeading()), idealRoute, speed);
					if (done) success = true;
				} catch(NoNewDataException nnde) {
					nnde.printStackTrace();
					Logger.logErr("Unable to get current heading, halting robot...");	
					mobilityPlane.stop();
				}
				
			}

			if (!done)
				ThreadControl.pause(this, NAV_CYCLE_PERIOD);
		}
		stop();
		Logger.log("Done going to " + endLoc + ", success=" + success);
		return success;
	}

	/**
	 * Stops the navigation process.  After calling this method, the 
	 * robot will stop moving.
	 */
	public synchronized void stop() {
		mobilityPlane.stop();
		done = true;
	}

	/**
	 * Calculates the proper velocity given the distance to the target and the desired velocity.
	 * As the distance decreases, the maximum velocity also decreases.
	 * 
	 * @param distance The distance to the target in meters
	 * @param desiredVelocity The desired velocity in meters per second.
	 * @return The proper velocity
	 */
	private double calcControlledVelocity(double distance, double desiredVelocity, double desiredHeading) {

		// The robot wants to make a very sharp turn; slow down
		if (Math.abs(desiredHeading) > maxSteeringAngle)
			return 0.6;

		// These numbers are tuned for the Traxxas Mobility Plane.
		// TODO: Generalize the controlled-velocity-generating-algorithm so it works with any mobility plane

		if (distance > 6)
			return desiredVelocity;
		else if (distance > 5)
			return (desiredVelocity > 1.5) ? 1.5 : desiredVelocity;
		else if (distance > 4)
			return (desiredVelocity > 1.0) ? 1.0 : desiredVelocity;
		else if (distance > 3)
			return (desiredVelocity > 0.7) ? 0.7 : desiredVelocity;
		else
			return (desiredVelocity > 0.5) ? 0.5 : desiredVelocity;
	}

	/**
	 * Calculates the next motion task that should be submitted to the MotionArbiter.
	 * The new motion tasks heading should ensure the robot continues to move towards the next way point.
	 * 
	 * @param currLoc The current location.
	 * @param currHeading The current heading in radians.
	 * @param idealRoute The ideal route that the robot should travel on.
	 * @param desiredSpeed The desired speed (in m/s) to travel towards the destination
	 * @return Whether the destination has been reached.
	 */
	private boolean doNextMotionTask(Location currLoc, double currHeading, 
			Line idealRoute, double desiredSpeed) 
	{
		boolean arrivedAtDest = false;
		Location idealLoc = idealRoute.getLocationClosestTo(currLoc);
		SpatialDivergence divergence = new SpatialDivergence(currLoc, idealLoc, idealRoute);

		// Save statistics in local variables.  This is used by areWeThereYet(...).
		double distanceToDestination = currLoc.distanceTo(idealRoute.getStopLoc());

		// If we are close enough to the destination location, stop.
		if (distanceToDestination < GPS_TARGET_RADIUS_METERS) {
			Logger.log("Destination " + idealRoute.getStopLoc() + " reached (distance = " + distanceToDestination + "m)!");
			arrivedAtDest = true;
			stop();
		} 

		// If the destination is some huge value, there must be an error, so stop.
		else if (distanceToDestination > 2000) {
			Logger.logErr("Invalid distance: Greater than 2km (" + distanceToDestination + "), stopping robot...");
			stop();
		} 

		// Figure out how to adjust the steering angle and speed to continue to move towards 
		// the destination.
		else {
			TargetDirection targetDirection = locateTarget(currLoc, currHeading, idealRoute.getStopLoc());

			/*  If the heading error is greater than MAJOR_HEADING_CORRECTION_THRESHOLD,
			 *  simply turn the robot at its maximum steering angle at a slow speed in
			 *  the correct direction. 
			 *  
			 *  positive --> robot is pointing too far to the right --> must turn left
			 *  negative --> robot is pointing too far to the left --> must turn right
			 */
			double headingError = targetDirection.getHeadingError();

			if (Math.abs(headingError) > MAJOR_HEADING_CORRECTION_THRESHOLD) {

				double steeringAngleMultiplier = Math.abs(headingError) / 0.6;
				if (steeringAngleMultiplier > 1)
					steeringAngleMultiplier = 1;

				double steeringAngle = maxSteeringAngle * steeringAngleMultiplier;

				/* A negative heading error means the robot is pointing too far to the
				 * left.  Thus, the steering angle should be negative to turn the robot
				 * right.
				 */
				if (headingError < 0)
					steeringAngle *= -1;
	
				mobilityPlane.set((float)steeringAngle, (float)headingCorrectionSpeed);

				Logger.log("Performing major correction:" +
						"\n\tHeading (radians): " + currHeading +
						"\n\tHeading error (radians): " + headingError + ", multiplier = " + steeringAngleMultiplier +
						"\n\tDistance to destination (m): " + targetDirection.getDistance() +
						"\n\tSteering angle cmd: " + steeringAngle +
						"\n\tSpeed cmd (m/s): " + headingCorrectionSpeed);
			} else {

				// Update the values of the PID controller
				totalError += headingError;
				double deltaHeadingErr = headingError - previousError;
				previousError = headingError;

				/*
				 * Since a positive heading error indicates that the robot should turn left, Kp > 0.
				 */
				double steeringAngle = Kp * headingError + Ki * totalError + Kp * deltaHeadingErr;

				/* Adjust the steering angle slightly to account for the spatial error.
				 * It was observed that the spatialError is typically between 0 and 1.5. 
				 * 
				 * positive spatial error --> The robot is *right* of the ideal path --> should turn left.
				 * negative spatial error --> The robot is *left* of the ideal path --> should turn right.
				 */
				double spatialError = divergence.getDivergence();
				double steeringAngleAdjusted = steeringAngle + SPATIAL_ERROR_FIX * spatialError;

				// Cap the steering angle to the min and max values.
				if (steeringAngleAdjusted > maxSteeringAngle)
					steeringAngleAdjusted = maxSteeringAngle;
				if (steeringAngleAdjusted < -maxSteeringAngle)
					steeringAngleAdjusted = -maxSteeringAngle;

				double speed = calcControlledVelocity(distanceToDestination, desiredSpeed, steeringAngleAdjusted);

				Logger.log("PID controller state:" + 
						"\n\tHeading (radians): " + currHeading +
						"\n\tHeading error (radians): " + headingError +
						"\n\tDistance to destination (m): " + distanceToDestination +
						"\n\tAbsolute divergence error (m): " + spatialError +
						"\n\tTotal error (radians): " + totalError +
						"\n\tDelta error (radians): " + deltaHeadingErr +
						"\n\t[Kp, Ki, Kd]: [" + Kp + ", " + Ki + ", " + Kd + "]" +
						"\n\tSteering angle cmd: " + steeringAngle + 
						"\n\tAdjusted Steering angle cmd:" + steeringAngleAdjusted +
						"\n\tSpeed cmd (m/s): " + speed);

				mobilityPlane.set((float)steeringAngle, (float)speed);
			}
		}
		return arrivedAtDest;
	}
}

