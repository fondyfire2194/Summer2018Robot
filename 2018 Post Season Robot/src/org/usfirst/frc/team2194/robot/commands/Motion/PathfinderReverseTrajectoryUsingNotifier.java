package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.PathfinderReverseNotifier;
import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;

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
		Robot.driveTrainCanBus.configDriveNominalOut(0, driveSide.both);
		Robot.driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		Robot.driveTrainCanBus.setStatusFramePeriod(2);
		Robot.driveTrainCanBus.setControlFramePeriod(2);
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
			Robot.simpleCSVLogger.init(Robot.chosenFile + "Rev", Robot.names, Robot.units);

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
		Robot.trajectoryRunning = false;
		Robot.driveTrainCanBus.leftDriveOut(0);
		Robot.driveTrainCanBus.rightDriveOut(0);
		Robot.driveTrainCanBus.setStatusFramePeriod(20);
		Robot.driveTrainCanBus.setControlFramePeriod(10);
		Robot.driveTrainCanBus.configOpenLoopAcceleration(.5);
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.close();
		PathfinderReverseNotifier.stopNotfier();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
