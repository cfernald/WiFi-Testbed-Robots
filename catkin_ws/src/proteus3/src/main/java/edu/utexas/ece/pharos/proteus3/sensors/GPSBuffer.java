package edu.utexas.ece.pharos.proteus3.sensors;

//import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;

// Import the messages
import proteus3_gps.GPSMsg;

import edu.utexas.ece.pharos.exceptions.NoNewDataException;
import edu.utexas.ece.pharos.exceptions.NoValidDataException;
import edu.utexas.ece.pharos.logger.Logger;

/**
 * Buffers incoming GPS measurements.
 *
 * @author Chien-Liang Fok
 */
public class GPSBuffer implements MessageListener<GPSMsg> {

	/**
	 * Holds the latest message received.
	 */
	private GPSMsg msg = null;

	/**
	 * Creates a GPS Buffer.
	 */
	public GPSBuffer() {
	}

	/**
	 * Determines whether a location is valid.  For now, it is hard-coded that valid
	 * locations have the following format:  
	 * The longitude must be -97.xxx degrees, and the latitude to be 30.xxx degrees.
	 * 
	 * @param msg The location message to check
	 * @return true if the location is valid.
	 */
	private boolean isValid(GPSMsg msg) {
		if (msg == null) return false; // a null location is inherently invalid

		int lonInt = (int)msg.getLongitude();

		// Constrain the longitude to be -97.xxx degrees
		if (lonInt != -97) {
			Logger.log("Rejecting location b/c it's longitude of " + lonInt + " != -97");
			return false;
		}

		// Constrain latitude to be 30.xxx degrees...
		int latInt = (int)msg.getLatitude();
		if (latInt != 30) {
			Logger.log("Rejecting location b/c it's latitude of " + latInt + " != 30");
			return false;
		}

		return true;
	}

	/**
	 * Obtains the current location.
	 * 
	 * @param maxAge The maximum age of the GPS location
	 * measurement in milliseconds.
	 * @return The current location
	 */
	public GPSMsg getLocation(long maxAge) throws NoNewDataException, NoValidDataException {
		if (msg == null)
			throw new NoNewDataException("No GPS data.");
		else {
			long gpsTimeSec = msg.getTimeSec();
			long gpsTimeUsec = msg.getTimeUsec();
			long timestamp = gpsTimeSec * 1000L + gpsTimeUsec / 1000L;
			long currTime = System.currentTimeMillis();
			long age = currTime - timestamp;
			
			Logger.log("currTime = " + currTime + ", gpsTimeSec = " + gpsTimeSec 
					+ ", gpsTimeUsec = " + gpsTimeUsec + ", timestamp = " + timestamp + ", age = " + age);
			
			if (age > maxAge)
				throw new NoNewDataException("Max GPS age exceeded (" + age + " > " + maxAge + ")");

			if (isValid(msg))
				return msg;
			else
				throw new NoValidDataException("GPS location of (" + msg.getLatitude() + ", " + msg.getLongitude() + ") not valid.");

		}
	}

	@Override
	public void onNewMessage(GPSMsg message) {
		if (isValid(message)) {
			this.msg = message;
			Logger.log("Received GPS message: " + msgToString(message));
		}
	}

	private String msgToString(GPSMsg message) {
		StringBuffer sb = new StringBuffer("(GPSMsg: ");
		sb.append("timeSec = " + message.getTimeSec());
		sb.append(", timeUsec = " + message.getTimeUsec());
		sb.append(", latitude = " + message.getLatitude());
		sb.append(", longitude = " + message.getLongitude());
		sb.append(", altitude = " + message.getAltitude());
		sb.append(", UtmE = " + message.getUtmE());
		sb.append( ", UtmN = " + message.getUtmN());
		sb.append(", quality = " + message.getQuality());
		sb.append(", numSats = " + message.getNumSats());
		sb.append(", Hdop = " + message.getHdop());
		sb.append(", Vdop = " + message.getVdop() + ")");
		return sb.toString();
	}
}
