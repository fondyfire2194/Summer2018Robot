package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HoldElevatorPositionMotionMagic extends Command {
	private double lastHoldPositionInches;
	private boolean firstTime;

	public HoldElevatorPositionMotionMagic() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.cubeHandler);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		RobotMap.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		RobotMap.elevatorMotor.selectProfileSlot(0, 0);
		// RobotMap.elevatorMotor.config_kF(0, Robot.prefs.getDouble("ElevatorMMKf",
		// 1.7), 0);
		// RobotMap.elevatorMotor.config_kP(0, Robot.prefs.getDouble("ElevatorMMKp", 0),
		// 0);
		// RobotMap.elevatorMotor.config_kI(0, Robot.prefs.getDouble("ElevatorMMKi", 0),
		// 0);
		// RobotMap.elevatorMotor.config_kD(0, Robot.prefs.getDouble("ElevatorMMKd", 0),
		// 0);
		RobotMap.elevatorMotor.configOpenloopRamp(0, 0);
		RobotMap.elevatorMotor.configClosedloopRamp(0, 0);
		RobotMap.elevatorMotor.configPeakOutputForward(1, 0);
		RobotMap.elevatorMotor.configPeakOutputReverse(-1, 0);
		RobotMap.elevatorMotor.configNominalOutputForward(0, 0);
		RobotMap.elevatorMotor.configNominalOutputReverse(0, 0);

		Robot.cubeHandler.holdPositionInches = Robot.cubeHandler.getElevatorPositionInches();
		lastHoldPositionInches = Robot.cubeHandler.holdPositionInches + .01;

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.cubeHandler.holdPositionInches != lastHoldPositionInches) {

			RobotMap.elevatorMotor.selectProfileSlot(0, 0);

			if (Robot.cubeHandler.moveIsUp) {
				RobotMap.elevatorMotor.config_kF(0,
						Robot.prefs.getDouble("ElevatorMMKf", CubeHandler.elevatorPrefsDefaults[0]), 0);
				RobotMap.elevatorMotor.config_kP(0,
						Robot.prefs.getDouble("ElevatorMMKp", CubeHandler.elevatorPrefsDefaults[1]), 0);
				RobotMap.elevatorMotor.config_kI(0,
						Robot.prefs.getDouble("ElevatorMMKi", CubeHandler.elevatorPrefsDefaults[2]), 0);
				RobotMap.elevatorMotor.config_kD(0,
						Robot.prefs.getDouble("ElevatorMMKd", CubeHandler.elevatorPrefsDefaults[3]), 0);
			} else {
				RobotMap.elevatorMotor.config_kF(0,
						Robot.prefs.getDouble("ElevatorDownMMKf", CubeHandler.elevatorPrefsDefaults[4]), 0);
				RobotMap.elevatorMotor.config_kP(0,
						Robot.prefs.getDouble("ElevatorDownMMKp", CubeHandler.elevatorPrefsDefaults[5]), 0);
				RobotMap.elevatorMotor.config_kI(0,
						Robot.prefs.getDouble("ElevatorDownMMKi", CubeHandler.elevatorPrefsDefaults[6]), 0);
				RobotMap.elevatorMotor.config_kD(0,
						Robot.prefs.getDouble("ElevatorDownMMKd", CubeHandler.elevatorPrefsDefaults[7]), 0);
			}
			Robot.cubeHandler.magicMotionElevator(Robot.cubeHandler.holdPositionInches,
					CubeHandler.ELEVATOR_POSITION_RATE);

			lastHoldPositionInches = Robot.cubeHandler.holdPositionInches;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
