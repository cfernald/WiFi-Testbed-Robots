package edu.utexas.ece.pharos.proteus3.apps.navigation;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import edu.utexas.ece.pharos.logger.FileLogger;
import edu.utexas.ece.pharos.logger.Logger;
import edu.utexas.ece.pharos.proteus3.sensors.CompassBuffer;
import edu.utexas.ece.pharos.proteus3.sensors.GPSBuffer;
import edu.utexas.ece.pharos.proteus3.mobilityPlanes.MobilityPlane;
import edu.utexas.ece.pharos.proteus3.mobilityPlanes.TraxxasMobilityPlane;
import edu.utexas.ece.pharos.proteus3.navigate.Location;
import edu.utexas.ece.pharos.proteus3.navigate.NavigateCompassGPS;

// Import the messages
import proteus3_compass.CompassMsg;
import proteus3_gps.GPSMsg;
import traxxas_node.AckermannDriveMsg;

/**
 * Moves a robot to a particular latitude and longitude coordinate
 * using information from compass and GPS sensors.
 *
 * @author Chien-Liang Fok
 */
public class MoveOutdoorCompassGPS extends AbstractNodeMain {
	
	final Location locB = new Location(30.38669,-97.72402);

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("proteus3/move_outdoor_compass_gps");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		System.setProperty ("PharosMiddleware.debug", "true");
		//final Log log = connectedNode.getLog();
		FileLogger flogger = new FileLogger("MoveOutdoorCompassGPS.log");
		Logger.setFileLogger(flogger);

		CompassBuffer compassBuffer = new CompassBuffer();
		GPSBuffer gpsBuffer = new GPSBuffer();

		Subscriber<CompassMsg> compassSubscriber = connectedNode.newSubscriber("/compass/measurement", CompassMsg._TYPE);
		compassSubscriber.addMessageListener(compassBuffer); 

		Subscriber<GPSMsg> gpsSubscriber = connectedNode.newSubscriber("/gps/measurement", GPSMsg._TYPE);
		gpsSubscriber.addMessageListener(gpsBuffer); 

		final Publisher<AckermannDriveMsg> traxxasPublisher = connectedNode.newPublisher("/traxxas_node/ackermann_drive", AckermannDriveMsg._TYPE);
		MobilityPlane mobilityPlane = new TraxxasMobilityPlane(traxxasPublisher);
		NavigateCompassGPS navCompGPS = new NavigateCompassGPS(mobilityPlane, compassBuffer, gpsBuffer);

		Location startLoc = null;
		Location endLoc = locB;
		double speed = 0.5;
		boolean success = navCompGPS.go(startLoc, endLoc, speed);
                if (success)
			Logger.log("Successfully arrived at destination.");
		else 
			Logger.log("Failed to arrive at destination.");
	}
}
