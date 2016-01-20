package edu.utexas.ece.pharos.proteus3.navigate;

/**
 * A spatial divergence measurement.  This is the
 * distance between a robot's current location and 
 * its ideal location on a line.
 *
 * @author Chien-Liang Fok
 */
public class SpatialDivergence {
	/**
	 * The time stamp of the divergence measurement.
	 */
	private long timeStamp;
	
	/**
	 * The ideal path along which the robot should traverse.
	 */
	private Line idealPath;
	
	/**
	 * The actual location of the robot.
	 */
	private Location currLoc;
	
	/**
	 * The ideal location that the robot should be at.  This depends on the 
	 * type of divergence being calculated.
	 */
	private Location idealLoc;
	
	/**
	 * A constructor that uses the current time as the timestamp.
	 * 
	 * @param currLoc The current location.
	 * @param idealLoc The ideal location.
	 * @param idealPath The ideal path along which the robot should travel.
	 */
	public SpatialDivergence(Location currLoc, Location idealLoc, Line idealPath) {
		this(System.currentTimeMillis(), currLoc, idealLoc, idealPath);
	}
	
	/**
	 * A constructor.
	 * 
	 * @param timeStamp The time stamp of the divergence measurement.
	 * @param currLoc The current location.
	 * @param idealLoc The ideal location.
	 * @param idealPath The ideal path along which the robot should travel.
	 */
	public SpatialDivergence(long timeStamp, Location currLoc, Location idealLoc, 
			Line idealPath) 
	{
		this.timeStamp = timeStamp;
		this.currLoc = currLoc;
		this.idealLoc = idealLoc;
		this.idealPath = idealPath;
	}
	
	/**
	 * @return The time stamp.
	 */
	public long getTimeStamp() {
		return timeStamp;
	}
	
	/**
	 * @return The ideal edge.
	 */
	public Line getIdealPath() {
		return idealPath;
	}
	
	/**
	 * @return The actual location of the robot.
	 */
	public Location getCurrLoc() {
		return currLoc;
	}
	
	/**
	 * @return The ideal location that the robot should be at.  This depends on the 
	 * type of divergence being calculated.
	 */
	public Location getIdealLoc() {
		return idealLoc;
	}
	
	/**
	 * @return The absolute value of the divergence in meters.
	 */
	public double getAbsoluteDivergence() {
		return Math.abs(getDivergence());
	}
	
	/**
	 * Returns the divergence (in meters) between the robot's current location and its
	 * ideal location.  A positive value means the robot is right of the line, whereas
	 * a negative value means the robot is left of the line.
	 * 
	 * @return The divergence in meters.
	 */
	public double getDivergence() {
		double divergence = currLoc.distanceTo(idealLoc);
		if (Line.isLocationLeftOf(getCurrLoc(), getIdealPath()))
			divergence *= -1;
		return divergence;
	}
	
	public String toString() {
		return "SpatialDivergence: " + timeStamp + "\t" + currLoc + "\t" + idealLoc 
			+ "\t" + getAbsoluteDivergence();
	}
}