package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;

public class PathfinderReverseNotifier {
	private static int passCounter = 0;
	private static int activeTrajectoryLength;
	private static double periodic_time = .02;
	private static double desired_heading;

	public static final class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			runReverseTrajectory();
		}
	}

	static Notifier _notifier = new Notifier(new PeriodicRunnable());
	private static double lastSegmentPosition;

	public static void startNotifier() {
		activeTrajectoryLength = Robot.activeLeftTrajectory.length();
		lastSegmentPosition = Robot.activeLeftTrajectory.get(activeTrajectoryLength - 1).position;
		passCounter = activeTrajectoryLength - 1;
		periodic_time = Robot.driveTrainCanBus.revLeftDf.getSegment().dt;
		_notifier.startPeriodic(periodic_time);
	}

	public static void stopNotfier() {
		_notifier.stop();
	}

	/*
	 * Fwd math
	 * 
	 * leftPct = DriveTrainCanBus.MINIMUM_START_PCT + left + turn;
	 * 
	 * rightPct = DriveTrainCanBus.MINIMUM_START_PCT + right - turn;
	 * 
	 * a + turn reading means robot is turned cw so need to speed up right and slow
	 * down left
	 * 
	 * Rev math
	 * 
	 * in reverse a + turn still means robot is turned cw but now we need to speed
	 * up left and slow down right so turn needs to be negated in equations Also
	 * output pct needs to be negated as does position feedback reading.
	 * 
	 * leftPct = -(DriveTrainCanBus.MINIMUM_START_PCT + left - turn);
	 * 
	 * rightPct = -(DriveTrainCanBus.MINIMUM_START_PCT + right + turn);
	 * 
	 */
	private static void runReverseTrajectory() {
		passCounter--;
		double left = Robot.driveTrainCanBus.revLeftDf.calculate(-Robot.driveTrainCanBus.getLeftFeet());
		double right = Robot.driveTrainCanBus.revRightDf.calculate(-Robot.driveTrainCanBus.getRightFeet());

		desired_heading = Pathfinder.r2d(Robot.driveTrainCanBus.revLeftDf.getHeading());

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - (-Robot.sensors.getGyroYaw()));
		double turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;

		double leftPct = DriveTrainCanBus.MINIMUM_START_PCT + left - turn;
		double rightPct = DriveTrainCanBus.MINIMUM_START_PCT + right + turn;

		Robot.driveTrainCanBus.leftDriveOut(-leftPct);
		Robot.driveTrainCanBus.rightDriveOut(-rightPct);

		if (passCounter > 1) {
			/*
			 * names = { "Step", "Left Cmd", "Left Ft", "Right Cmd ", "Right Ft",
			 * "Angle Cmd", "Angle", "LeftSegVel", "left", "ActLeftVel", "RightSegVel",
			 * "right", "ActRightVel", "turn" ,"battery"};
			 * 
			 */
			Robot.simpleCSVLogger.writeData((double) passCounter,
					lastSegmentPosition - Robot.driveTrainCanBus.revLeftDf.getSegment().position,
					-Robot.driveTrainCanBus.getLeftFeet(),
					lastSegmentPosition - Robot.driveTrainCanBus.revRightDf.getSegment().position,
					-Robot.driveTrainCanBus.getRightFeet(), Pathfinder.boundHalfDegrees(desired_heading),
					-Robot.sensors.getGyroYaw(),
					Robot.driveTrainCanBus.revLeftDf.getSegment().velocity / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					leftPct, Robot.driveTrainCanBus.getLeftFeetPerSecond() / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					Robot.driveTrainCanBus.revRightDf.getSegment().velocity / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					rightPct, Robot.driveTrainCanBus.getRightFeetPerSecond() / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					turn, Robot.powerPanel.getVoltage());
		}
	}
}
