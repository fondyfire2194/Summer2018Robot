package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.PathfinderNotifier;
import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PathfinderTrajectoryUsingNotifier extends Command {
	private double startTime;
	boolean myExchangeWheelTrajectories;
	private int scanCounter;

	public PathfinderTrajectoryUsingNotifier() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrainCanBus.resetEncoders();
		// Robot.sensors.resetGyro();
		Robot.driveTrainCanBus.leftSideStopped = false;
		Robot.driveTrainCanBus.rightSideStopped = false;
		Robot.driveTrainCanBus.configOpenLoopAcceleration(0);
		Robot.driveTrainCanBus.configDriveNominalOut(0, driveSide.both);
		Robot.driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		Robot.driveTrainCanBus.setStatusFramePeriod(2);
		Robot.driveTrainCanBus.setControlFramePeriod(2);
		double P = Robot.activeTrajectoryGains[0];
		double I = 0;
		double D = Robot.activeTrajectoryGains[1];
		double V = 1 / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC;
		double A = Robot.activeTrajectoryGains[2];

		Robot.driveTrainCanBus.leftDf.setTrajectory(Robot.activeLeftTrajectory);
		Robot.driveTrainCanBus.rightDf.setTrajectory(Robot.activeRightTrajectory);
		Robot.driveTrainCanBus.leftPositionTargetFt = Robot.activeLeftTrajectory
				.get(Robot.activeLeftTrajectory.length() - 1).position;
		Robot.driveTrainCanBus.rightPositionTargetFt = Robot.activeRightTrajectory
				.get(Robot.activeRightTrajectory.length() - 1).position;

		Robot.driveTrainCanBus.leftDf.configurePIDVA(P, I, D, V, A);
		Robot.driveTrainCanBus.rightDf.configurePIDVA(P, I, D, V, A);

		Robot.driveTrainCanBus.leftDf.reset();
		Robot.driveTrainCanBus.rightDf.reset();
		Robot.trajectoryRunning = true;
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.init(Robot.chosenFile, Robot.names, Robot.units);
		scanCounter = 0;
		startTime = Timer.getFPGATimestamp();
		PathfinderNotifier.startNotifier();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		scanCounter++;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return scanCounter > 20 && Robot.driveTrainCanBus.leftDf.isFinished()
				&& Robot.driveTrainCanBus.rightDf.isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.trajectoryRunning = false;
		Robot.driveTrainCanBus.leftDriveOut(0);
		Robot.driveTrainCanBus.rightDriveOut(0);
		Robot.driveTrainCanBus.setStatusFramePeriod(20);
		Robot.driveTrainCanBus.setControlFramePeriod(10);
		Robot.driveTrainCanBus.configOpenLoopAcceleration(.5);
		SmartDashboard.putNumber("Trajectory Time", Timer.getFPGATimestamp() - startTime);
		PathfinderNotifier.stopNotfier();
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
