package edu.utexas.ece.pharos.proteus3.mobilityPlanes;

/**
 * Defines the interface that all mobility planes must implement.
 * 
 * @author Chien-Liang Fok
 */
public abstract class MobilityPlane {

  /**
   * Sets the steering angle in radians.  Zero is straight ahead, positive is to
   * the left, and negative is to the right.
   */
  public abstract void setSteeringAngle(float steeringAngle);

  /**
   * Sets the speed in m/s.
   */
  public abstract void setSpeed(float speed);

  /**
   * Sets the steering angle and speed.
   * 
   * @param steeringAngle  The steering angle in radians.
   * @param speed The speed in m/s.
   */
  public void set(float steeringAngle, float speed) {
	  setSteeringAngle(steeringAngle);
	  setSpeed(speed);
  }

  /**
   * Stops the robot.
   */
  public abstract void stop();
  
  /**
   * Kills the robot.  After calling this, no further commands
   * can be sent to the robot.
   */
  public abstract void kill();
}
