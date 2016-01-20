package edu.utexas.ece.pharos.proteus3.sensors;

import org.ros.message.MessageListener;

import edu.utexas.ece.pharos.exceptions.NoNewDataException;
import edu.utexas.ece.pharos.logger.Logger;

import proteus3_compass.CompassMsg;

/**
 * Buffers incoming compass measurements.
 *
 * @author Chien-Liang Fok
 */
public class CompassBuffer implements MessageListener<CompassMsg> {
	
	/**
	 * The latest compass measurement.
	 */
	private CompassMsg msg = null;
	
	/**
	 * The time when the latest compass measurement was received.
	 */
	private long timestamp = -1;

	/**
	 * The constructor.
	 */
	public CompassBuffer() {
	}
	
	/**
	 * Obtains the current compass measurement.
	 * 
	 * @param maxAge The maximum age of the GPS location
	 * measurement in milliseconds.
	 * @return The current compass measurement
	 */
	public CompassMsg getHeading(long maxAge) throws NoNewDataException {
		if (msg == null || timestamp == -1)
			throw new NoNewDataException("No comapss data.");
		else {
			long age = System.currentTimeMillis() - timestamp;
			if (age > maxAge)
				throw new NoNewDataException("Max compass data age exceeded (" + age + " > " + maxAge + ")");
			else {
				return msg;
			}
		}
	}

	@Override
	public void onNewMessage(CompassMsg message) {
		this.msg = message;
		this.timestamp = System.currentTimeMillis();
		Logger.log("Received compass message: (CompassMsg: heading = " + message.getHeading()
				+ ", pitch = " + message.getPitch()
				+ ", roll = " + message.getRoll() + ")");
	}
}
