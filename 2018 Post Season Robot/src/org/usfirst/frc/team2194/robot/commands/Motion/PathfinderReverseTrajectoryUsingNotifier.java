package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.PathfinderReverseNotifier;
import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PathfinderReverseTrajectoryUsingNotifier extends Command {

	public PathfinderReverseTrajectoryUsingNotifier() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainCanBus);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrainCanBus.resetEncoders();
		// Robot.sensors.resetGyro();
		Robot.driveTrainCanBus.configOpenLoopAcceleration(0);
		Robot.driveTrainCanBus.configDriveNominalOut(0, driveSide.both);
		Robot.driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		double P = Robot.activeTrajectoryGains[0];
		double I = 0;
		double D = Robot.activeTrajectoryGains[1];
		double V = 1 / DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC;
		double A = Robot.activeTrajectoryGains[2];

		Robot.driveTrainCanBus.revLeftDf.setTrajectory(Robot.activeLeftTrajectory);
		Robot.driveTrainCanBus.revRightDf.setTrajectory(Robot.activeRightTrajectory);

		Robot.driveTrainCanBus.revLeftDf.configurePIDVA(P, I, D, V, A);
		Robot.driveTrainCanBus.revRightDf.configurePIDVA(P, I, D, V, A);

		Robot.driveTrainCanBus.revLeftDf.reset();
		Robot.driveTrainCanBus.revRightDf.reset();

		Robot.trajectoryRunning = true;
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.init("Trajectory", Robot.chosenFile + "Rev", Robot.names, Robot.units);

		PathfinderReverseNotifier.startNotifier();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.driveTrainCanBus.revLeftDf.isFinished() && Robot.driveTrainCanBus.revRightDf.isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		PathfinderReverseNotifier.stopNotfier();
		Robot.trajectoryRunning = false;
		Robot.driveTrainCanBus.leftDriveOut(0);
		Robot.driveTrainCanBus.rightDriveOut(0);
		Robot.driveTrainCanBus.configOpenLoopAcceleration(.5);
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
