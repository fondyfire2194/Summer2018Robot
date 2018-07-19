/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.SD;

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

	public double loopOutput;
	public boolean orientClockwise;

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
		double temp = 0;
		if (Math.abs(output) < DriveTrainCanBus.MINIMUM_START_PCT) {
			temp = DriveTrainCanBus.MINIMUM_START_PCT;
			if (output < 0)
				temp = -temp;
		} else
			temp = output;
		loopOutput = temp;
		Robot.driveTrainCanBus.leftDriveOut(temp);
		Robot.driveTrainCanBus.rightDriveOut(-temp);
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
		return (Math.abs(getError()) <= 3);
	}

	public boolean closeToPosition() {
		return (Math.abs(getError()) <= 5);
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public void updateStatus() {
		SmartDashboard.putBoolean("RotateInPos", inPosition());
		SD.putN1("Setpoint", getSetpoint());
		SmartDashboard.putBoolean("Rotate Enabled?", isEnabled());
		SD.putN1("Orient Error", getError());

		SD.putN("Rot Output", loopOutput);

	}
}
