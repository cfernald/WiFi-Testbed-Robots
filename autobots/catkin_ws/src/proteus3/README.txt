To compile:

$ $(rosstack find rosjava_core)/gradlew installApp

To run:

$ ./build/install/proteus3/bin/proteus3 [name of main class file]

where [name of main class file] may be:
 - edu.utexas.ece.pharos.proteus3.apps.navigation.MoveOutdoorCompassGPS 
 - edu.utexas.ece.pharos.proteus3.apps.tests.CompassLogger

To clean:

$ $(rosstack find rosjava_core)/gradlew clean

Note 1: 
The class edu.utexas.ece.pharos.proteus3.sensors.GPSBuffer 
requires the system time to be accurate since it compares 
the system time to the GPS time to determine the age of the 
latest GPS measurement.  Calibrate the system time using
NTP by executing the following command:

$ sudo ntpdate ntp.ubuntu.com

Note 2:
The pharos node relies on various java messages.  Create them 
by reinstalling the 'rosjava_messages' node as follows:

$ roscd rosjava_messages
$ ../gradlew clean
$ ../gradlew install

Note 3:
To install dependent jars:

$ mvn install:install-file -Dfile=jars/jcommon-1.0.17.jar -DgroupId=jcommon -DartifactId=jcommon -Dversion=1.0.17 -Dpackaging=jar

$ mvn install:install-file -Dfile=jars/jfreechart-1.0.14.jar -DgroupId=jfreechart -DartifactId=jfreechart -Dversion=1.0.14 -Dpackaging=jar

After installing the jars, add the following jar files to the Eclipse project:
 - ~/.m2/repository/jfreechart/jfreechart/1.0.14/jfreechart-1.0.14.jar
 - ~/.m2/repository/jcommon/jcommon/1.0.17/jcommon-1.0.17.jar

