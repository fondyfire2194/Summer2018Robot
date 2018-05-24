package org.usfirst.frc.team2194.robot;

/*The thread that starts here will monitor the robot for driving into the Power Up switch 
 * as well as looking for collisions and suddden stops.
 *  The cube elevator will also be monitored since it is a vulnerable mechanism
 *  
 *  Driving into the switch (or anything else) will be monitored by looking at
 *  a combination of the wheel encoders not changing for a period while the drive are being
 * commanded to move. This will need to be filtered so that at a move start
 *  if doesn't trip unnecissarily.
 * The navX has a collision detect piece of code that can be impimented also.
 *     
 *  We could also do the intake / outtake current during cube transfers monitoring here
 *  The elevator motor current and temperature can be monitored for jams or other bad situations
 * 
 * 
 * 
 */
public class RobotDriveMonitor {

	public static final int UPDATE_RATE_MS = 50;
	boolean something_to_do = false;
	private int lastLeftEncoderPosition;
	private int lastRightEncoderPosition;
	private int DRIVE_ENCODER_STOPPED_BAND = 10;
	private double DRIVE_MOTOR_POWER_STOPPED_BAND = .005;
	private int leftStoppedCounter;
	private int rightStoppedCounter;
	private double lastRobotPositionFt;
	private double xLastChange;
	private double yLastChange;

	public RobotDriveMonitor() {

		// Reset give up flag
		something_to_do = Robot.driveTrainCanBus.runStalledDetect;

		// Kick off monitor in brand new thread.
		// Thanks to Team 254 and Robot Casserole for an example of how to do this!
		Thread driveMonitorThread = new Thread(new Runnable() {
			@Override
			public void run() {
				try {
					while (something_to_do) {
						periodicUpdate();
						Thread.sleep(UPDATE_RATE_MS);
					}
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});

		// Set up thread properties and start it off
		driveMonitorThread.setName("RobotDrivesMonitor");
		driveMonitorThread.setPriority(Thread.MIN_PRIORITY + 1);
		driveMonitorThread.start();
	}

	private void periodicUpdate() {
		if (Robot.driveTrainCanBus.runStalledDetect) {
			if (Math.abs(RobotMap.driveLeftMotorA.getMotorOutputPercent()) > DRIVE_MOTOR_POWER_STOPPED_BAND) {
				if (Math.abs(Robot.driveTrainCanBus.getLeftEncoder()
						- lastLeftEncoderPosition) < DRIVE_ENCODER_STOPPED_BAND) {
					leftStoppedCounter++;
					lastLeftEncoderPosition = Robot.driveTrainCanBus.getLeftEncoder();
				} else
					leftStoppedCounter = 0;
			}
			Robot.driveTrainCanBus.leftSideStopped = leftStoppedCounter > 2;

			if (Math.abs(RobotMap.driveRightMotorA.getMotorOutputPercent()) > DRIVE_MOTOR_POWER_STOPPED_BAND) {

				if (Math.abs(Robot.driveTrainCanBus.getRightEncoder()
						- lastRightEncoderPosition) < DRIVE_ENCODER_STOPPED_BAND) {
					rightStoppedCounter++;
					lastRightEncoderPosition = Robot.driveTrainCanBus.getRightEncoder();
				} else
					rightStoppedCounter = 0;
			}
			Robot.driveTrainCanBus.rightSideStopped = rightStoppedCounter > 2;

		}
		/*
		 * Keep (approximate) track of robot position in X (up field) and Y (across
		 * field) through computing distance moved (encoders) and angle turned (gyro).
		 * 
		 * Use formula Xchange = encoder distance change * cosine gyro angle.
		 * 
		 * and Y change = encoder distance change * - sine gyro angle
		 * 
		 * 
		 * 
		 */
		double distanceChange = Robot.driveTrainCanBus.getRobotPositionFeet() - lastRobotPositionFt;

		xLastChange = distanceChange * Math.cos(Math.toRadians(-Robot.sensors.getGyroYaw()));
		Robot.xPosition += xLastChange;

		yLastChange = -distanceChange * Math.sin(Math.toRadians(-Robot.sensors.getGyroYaw()));
		Robot.yPosition += yLastChange;

		lastRobotPositionFt = Robot.driveTrainCanBus.getRobotPositionFeet();

	}

}
