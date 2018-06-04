/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.SD;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author John
 */
public class RobotRotate extends PIDSubsystem {

	private static final double Kp = .01;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	private static final double toleranceAngle = 2;
	private static final int toleranceBuffer = 5;
	private boolean useLeftSide;
	private boolean useRightSide;

	// Initialize your subsystem here
	public RobotRotate() {
		super("RobotRotate", Kp, Ki, Kd);
		getPIDController().setInputRange(-180, 180);
		getPIDController().setOutputRange(-1, 1);
		getPIDController().setContinuous();
		getPIDController().disable();
		getPIDController().setAbsoluteTolerance(toleranceAngle);
		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable() - Enables the PID controller.
	}

	public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}

	@Override
	public PIDController getPIDController() {
		// TODO Auto-generated method stub
		return super.getPIDController();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	protected double returnPIDInput() {
		return Robot.sensors.imu.getYaw();
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	@Override
	protected void usePIDOutput(double output) {
		// RobotMap.driveLeftMotorA.set(ControlMode.PercentOutput,output);
		// RobotMap.driveRightMotorA.set(ControlMode.PercentOutput,-output);

		if (!Robot.closeDriveSpeedLoop) {
			RobotMap.driveLeftMotorA.set(ControlMode.PercentOutput, output);
			RobotMap.driveRightMotorA.set(ControlMode.PercentOutput, -output);
		} else {
			RobotMap.driveLeftMotorA.set(ControlMode.Velocity, output * Robot.driveTrainCanBus.MAX_ENC_CTS_PER_100MS);
			RobotMap.driveRightMotorA.set(ControlMode.Velocity, -output * Robot.driveTrainCanBus.MAX_ENC_CTS_PER_100MS);
		}
		// // Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
	}

	public void enablePID() {
		getPIDController().enable();
	}

	public void disablePID() {
		getPIDController().disable();
	}

	@Override
	public void setSetpoint(double setpoint) {
		getPIDController().setSetpoint(setpoint);
	}

	public void setMaxOut(double speed) {
		getPIDController().setOutputRange(-speed, speed);
	}

	public double getKp() {
		return getPIDController().getP();
	}

	public double getError() {
		return getPIDController().getError();
	}

	public boolean inPosition() {
		return (Math.abs(getError()) < 3);
	}

	public boolean closeToPosition() {
		return (Math.abs(getError()) < 5);
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public void updateStatus() {
		SmartDashboard.putBoolean("RotateInPos", inPosition());
		SmartDashboard.putBoolean("LeftSide", useLeftSide);
		SmartDashboard.putBoolean("RightSide", useRightSide);
		SD.putN1("Setpoint", getSetpoint());
		SmartDashboard.putBoolean("Rotate Enabled?", isEnabled());
		SD.putN1("Orient Error", getError());

	}
}
