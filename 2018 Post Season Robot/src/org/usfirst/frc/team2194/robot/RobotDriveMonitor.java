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

	// public static final int UPDATE_RATE_MS = 20;
	private int lastLeftEncoderPosition;
	private int lastRightEncoderPosition;
	private int DRIVE_ENCODER_STOPPED_BAND = 10;
	private double DRIVE_MOTOR_POWER_STOPPED_BAND = 1;
	private int leftStoppedCounter;
	private int rightStoppedCounter;
	private double lastXRobotPositionFt;
	private double lastYRobotPositionFt;
	private double distanceChange;
	private int leftEncoderStoppedCounter;
	private int rightEncoderStoppedCounter;

	public RobotDriveMonitor() {
	}

	public boolean getLeftEncoderStopped() {
		if (Math.abs(RobotMap.driveLeftMotorA.getSelectedSensorVelocity(0)) < 10)
			leftEncoderStoppedCounter++;
		else
			leftEncoderStoppedCounter = 0;
		return leftEncoderStoppedCounter > 10;
	}

	public boolean getRightEncoderStopped() {
		if (Math.abs(RobotMap.driveRightMotorA.getSelectedSensorVelocity(0)) < 10)
			rightEncoderStoppedCounter++;
		else
			rightEncoderStoppedCounter = 0;
		return rightEncoderStoppedCounter > 10;
	}

	public boolean getLeftDriveStopped() {
		if (Robot.driveTrainCanBus.runStalledDetect) {
			if (Math.abs(RobotMap.driveLeftMotorA.getMotorOutputPercent()) > DRIVE_MOTOR_POWER_STOPPED_BAND) {
				if (Math.abs(Robot.driveTrainCanBus.getLeftEncoder()
						- lastLeftEncoderPosition) < DRIVE_ENCODER_STOPPED_BAND) {
					leftStoppedCounter++;
					lastLeftEncoderPosition = Robot.driveTrainCanBus.getLeftEncoder();
				} else
					leftStoppedCounter = 0;
			}

		}
		return false;
		// return leftStoppedCounter > 2;
	}

	public boolean getRightDriveStopped() {

		if (Math.abs(RobotMap.driveRightMotorA.getMotorOutputPercent()) > DRIVE_MOTOR_POWER_STOPPED_BAND) {
			if (Math.abs(
					Robot.driveTrainCanBus.getRightEncoder() - lastRightEncoderPosition) < DRIVE_ENCODER_STOPPED_BAND) {
				rightStoppedCounter++;
				lastRightEncoderPosition = Robot.driveTrainCanBus.getRightEncoder();
			} else
				rightStoppedCounter = 0;
		}
		return false;
		// return rightStoppedCounter > 2;

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

	public double updateXPosition() {
		double latchPositionReading = Robot.driveTrainCanBus.getRobotPositionFeet();
		distanceChange = latchPositionReading - lastXRobotPositionFt;
		lastXRobotPositionFt = latchPositionReading;
		return distanceChange * Math.cos(Math.toRadians(-Robot.sensors.getGyroYaw()));
	}

	public double updateYPosition() {
		double latchPositionReading = Robot.driveTrainCanBus.getRobotPositionFeet();
		distanceChange = latchPositionReading - lastYRobotPositionFt;
		lastYRobotPositionFt = latchPositionReading;
		return -distanceChange * Math.sin(Math.toRadians(-Robot.sensors.getGyroYaw()));
	}
}
