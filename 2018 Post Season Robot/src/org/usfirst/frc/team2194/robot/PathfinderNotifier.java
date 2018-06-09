package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

public class PathfinderNotifier {
	private static double thisTime;
	private static double timeDifference;
	private static double lastTime;
	private static double timeSum;
	public static double timeAverage;
	public static boolean isRunning;
	public static int segmentCounter = 0;
	public static double minTime = 9999;
	public static double maxTime = 0;

	public static final class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			runTrajectory();
		}
	}

	static Notifier _notifier = new Notifier(new PeriodicRunnable());
	private static int activeTrajectoryLength;
	private static double periodic_time = .02;
	private static double desired_heading;

	public static void startNotifier() {
		minTime = 999;
		maxTime = 0;
		timeAverage = 0;
		segmentCounter = 0;
		timeSum = 0;
		thisTime = 0;
		lastTime = 0;
		activeTrajectoryLength = Robot.activeLeftTrajectory.length();
		periodic_time = Robot.driveTrainCanBus.leftDf.getSegment().dt;
		_notifier.startPeriodic(periodic_time);
	}

	public static void stopNotfier() {
		_notifier.stop();
	}

	private static void runTrajectory() {
		/*
		 * Pathfinder calculations to arrive at pct output to left and right motors
		 * 
		 * Note the gyro yaw is opposite in sign to the Pathfinder heading so it needs
		 * to be negated before it is subtracted to get the angular error. A positive
		 * error means turn left so speed up the right side and slow down the left The
		 * negative gain multiplier causes (right - turn) and (left + turn) to do this
		 * 
		 * public double calculate(double distance_covered)
		 * 
		 * { if (segment <trajectory.length()) {
		 * 
		 * Trajectory.Segment seg = trajectory.get(segment);
		 * 
		 * double error = seg.position - distance_covered;
		 * 
		 * double calculated_value = kp* error + // Proportional
		 *
		 * kd * ((error - last_error) / seg.dt) + //Derivative
		 * 
		 * (kv * seg.velocity + ka * seg.acceleration); // V and A Terms
		 *
		 * last_error = error; heading = seg.heading; segment++;
		 *
		 * return calculated_value;
		 * 
		 * }
		 * 
		 * else return 0; }
		 *
		 */
		segmentCounter++;

		double left = Robot.driveTrainCanBus.leftDf.calculate(Robot.driveTrainCanBus.getLeftFeet());
		double right = Robot.driveTrainCanBus.rightDf.calculate(Robot.driveTrainCanBus.getRightFeet());
		desired_heading = Pathfinder.r2d(Robot.driveTrainCanBus.leftDf.getHeading());

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - (-Robot.sensors.getGyroYaw()));
		double turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
		/*
		 * see Reverse Pathfinder Notifier for explanations of following equations
		 * 
		 * 
		 */
		double leftPct = DriveTrainCanBus.MINIMUM_START_PCT + (left + turn);
		double rightPct = DriveTrainCanBus.MINIMUM_START_PCT + (right - turn);

		if (segmentCounter < activeTrajectoryLength - 1) {
			/*
			 * write linear and angular data to file
			 * 
			 * names = { "Step", "Left Cmd", "Left Ft", "Right Cmd ", "Right Ft",
			 * "Angle Cmd", "Angle", "LeftSegVel", "left", "ActLeftVel", "RightSegVel",
			 * "right", "ActRightVel", "turn" };
			 * 
			 */

			Robot.simpleCSVLogger.writeData((double) segmentCounter,
					Robot.driveTrainCanBus.leftDf.getSegment().position, Robot.driveTrainCanBus.getLeftFeet(),
					Robot.driveTrainCanBus.rightDf.getSegment().position, Robot.driveTrainCanBus.getRightFeet(),
					Pathfinder.boundHalfDegrees(desired_heading), -Robot.sensors.getGyroYaw(),
					Robot.driveTrainCanBus.leftDf.getSegment().velocity / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, left,
					Robot.driveTrainCanBus.getLeftFeetPerSecond() / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					Robot.driveTrainCanBus.rightDf.getSegment().velocity / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, right,
					Robot.driveTrainCanBus.getRightFeetPerSecond() / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, turn);
		}

		Robot.driveTrainCanBus.leftDrivePctOut(leftPct);
		Robot.driveTrainCanBus.rightDrivePctOut(rightPct);

		thisTime = Timer.getFPGATimestamp();
		timeDifference = thisTime - lastTime;
		timeSum += timeDifference;
		lastTime = thisTime;
		if (segmentCounter >= 10) {
			timeAverage = timeDifference;
			if (timeAverage < minTime)
				minTime = timeAverage;
			if (timeAverage > maxTime)
				maxTime = timeAverage;
			timeSum = 0;
			SmartDashboard.putNumber("Time", timeAverage);
		}
	}
}
