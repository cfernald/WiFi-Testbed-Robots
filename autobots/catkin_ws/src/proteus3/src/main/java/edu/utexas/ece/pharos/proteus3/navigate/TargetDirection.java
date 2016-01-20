package edu.utexas.ece.pharos.proteus3.navigate;


/**
 * Encapsulates the distance to a particular target, and the change
 * in heading necessary for the robot to point toward the target.
 * 
 * @author Chien-Liang Fok
 * @see edu.utexas.ece.pharos.proteus3.navigate.Navigate
 */
public class TargetDirection {
	private double distance; // the distance in meters
	private double headingError; // the heading error in radians
	
	public TargetDirection(double distance, double headingError) {
		this.distance = distance;
		this.headingError = headingError;
	}
	
	/**
	 * Returns the distance to the target in meters.
	 * @return the distance to the target in meters.
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Returns the heading error in radians.  A negative heading error
	 * indicates that the robot should turn right.
	 * 
	 * @return the heading error in radians.
	 */
	public double getHeadingError() {
		return headingError;
	}
	
	public String toString() {
		return "Distance to Target: " + distance + " meters; Heading error: " + headingError + " radians";
	}
}