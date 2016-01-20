package edu.utexas.ece.pharos.proteus3.mobilityPlanes;

import org.ros.node.topic.Publisher;

import traxxas_node.AckermannDriveMsg;
import edu.utexas.ece.pharos.logger.Logger;
import edu.utexas.ece.pharos.util.ThreadControl;

/**
 * Provides access to the Traxxas mobility plane of the Proteus III
 * robot.
 * 
 * @author Chien-Liang Fok
 */
public class TraxxasMobilityPlane extends MobilityPlane implements Runnable {
	/**
	 * The cycle period in milliseconds.
	 */
	public static final long CYCLE_PERIOD = 333;
	
	final Publisher<AckermannDriveMsg> traxxasPublisher;
	
	private boolean done = false;
	
	private AckermannDriveMsg msg;
	
	/**
	 * The constructor.
	 */
	public TraxxasMobilityPlane(final Publisher<AckermannDriveMsg> traxxasPublisher) {
		this.traxxasPublisher = traxxasPublisher;
		msg = traxxasPublisher.newMessage();
		new Thread(this).start();
	}

	/**
	 * Sets the steering angle in radians.  Zero is straight ahead, positive is to
	 * the left, and negative is to the right.
	 */
	@Override
	public void setSteeringAngle(float steeringAngle) {
		msg.setSteeringAngle((float)Math.toDegrees(steeringAngle));
		Logger.log("Steering angle set to " + steeringAngle + " (" + msg.getSteeringAngle() + " degrees)");
	}

	/**
	 * Sets the speed in m/s.
	 */
	@Override
	public void setSpeed(float speed) {
		Logger.log("Speed set to " + speed + "m/s");
		msg.setSpeed(speed);
	}

	@Override
	public void stop() {
		Logger.log("Stop called.");
		msg.setSteeringAngle(0f);
		msg.setSpeed(0f);
	}
	
	@Override
	public void kill() {
		Logger.log("Kill called.");
		done = true;
	}

	@Override
	public void run() {
		while (!done) {
			Logger.log("Publishing command: Speed = " + msg.getSpeed() 
					+ ", Steering angle = " + msg.getSteeringAngle());
			traxxasPublisher.publish(msg);
			// Pause for a moment before repeating
			if (!done)
				ThreadControl.pause(this, CYCLE_PERIOD);
		}
	}
}
