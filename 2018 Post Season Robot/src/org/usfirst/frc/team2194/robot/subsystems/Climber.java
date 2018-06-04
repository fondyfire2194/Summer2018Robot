package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

	TalonSRX climber = RobotMap.climberMotorA;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());

	}

	public void driveClimber(double input) {
		climber.set(ControlMode.PercentOutput, input);
	}

	public void stopClimber() {
		climber.set(ControlMode.Disabled, 0);
	}

	public void updateStatus() {
		// SmartDashboard.putNumber("Climber Amps", climber.getOutputCurrent());
	}
}
