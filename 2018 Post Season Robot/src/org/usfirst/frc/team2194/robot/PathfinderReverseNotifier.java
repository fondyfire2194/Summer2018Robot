package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import com.ctre.phoenix.motorcontrol.ControlMode;

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

	public static void startNotifier() {
		activeTrajectoryLength = Robot.activeLeftTrajectory.length();
		passCounter = activeTrajectoryLength - 1;
		periodic_time = Robot.driveTrainCanBus.revLeftDf.getSegment().dt;
		_notifier.startPeriodic(periodic_time);
	}

	public static void stopNotfier() {
		_notifier.stop();
	}

	/*
	 * Look at trying to run a trajectory with the robot moving in reverse from the
	 * finishing robot location of the same trajectory run forward. The normal
	 * forward equations are derived from
	 * 
	 * angle difference = desired heading - gyroYaw
	 * 
	 * and turn = turn gain * (- angle) difference
	 * 
	 * Pathfinder has the robot turning left as positive which results in a - yaw in
	 * navX. So angle difference will be + for a left turn but the turn value will
	 * be a - value (because of the - sign in the gain multiplication) when the
	 * robot needs to turn more left which means speeding up the right side and
	 * slowing down the left.
	 * 
	 * leftPct = left + turn and rightPct = right - turn
	 * 
	 * So moving in a forward direction, a - turn value (+ angle difference) speeds
	 * up the right and slows down the left.
	 * 
	 * Now look at the robot running backwards. The left and right drives are
	 * receiving negative values so if we keep the same equations as forward, adding
	 * a - turn value will speed up that side and subtracting it will slow it down.
	 * 
	 * So running backwards, a -turn value will speed up the left and slow down the
	 * right. This will cause the robot to turn in the correct direction.
	 * 
	 * The equations for drive output remain the same in both directions.
	 */
	private static void runReverseTrajectory() {
		passCounter--;
		double left = Robot.driveTrainCanBus.revLeftDf.calculate(-Robot.driveTrainCanBus.getLeftFeet());
		double right = Robot.driveTrainCanBus.revRightDf.calculate(-Robot.driveTrainCanBus.getRightFeet());

		desired_heading = Pathfinder.r2d(Robot.driveTrainCanBus.revLeftDf.getHeading());

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - (-Robot.sensors.getGyroYaw()));
		double turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;

		RobotMap.driveLeftMotorA.set(ControlMode.PercentOutput, -DriveTrainCanBus.MINIMUM_START_PCT - left + turn);
		RobotMap.driveRightMotorA.set(ControlMode.PercentOutput, -DriveTrainCanBus.MINIMUM_START_PCT - right - turn);

		if (Robot.createTrajectoryDebugFile && passCounter > 1) {
			/*
			 * names = { "Step", "Left Cmd", "Left Ft", "Right Cmd ", "Right Ft",
			 * "Angle Cmd", "Angle", "LeftSegVel", "left", "ActLeftVel", "RightSegVel",
			 * "right", "ActRightVel", "turn" };
			 * 
			 */
			Robot.simpleCSVLogger.writeData((double) passCounter,
					Robot.driveTrainCanBus.revLeftDf.getSegment().position, -Robot.driveTrainCanBus.getLeftFeet(),
					Robot.driveTrainCanBus.revRightDf.getSegment().position, -Robot.driveTrainCanBus.getRightFeet(),
					desired_heading, -Robot.sensors.getGyroYaw(),
					Robot.driveTrainCanBus.revLeftDf.getSegment().velocity / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					left, Robot.driveTrainCanBus.getLeftFeetPerSecond() / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					Robot.driveTrainCanBus.revRightDf.getSegment().velocity / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					right, Robot.driveTrainCanBus.getRightFeetPerSecond() / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC,
					turn);
		}

	}
}
