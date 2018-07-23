package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.SD;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.HoldElevatorPositionMotionMagic;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeHandler extends Subsystem {
	public static double ENCODER_COUNTS_PER_INCH = 341.3;
	public static double ELEVATOR_MIN_HEIGHT = -.50;
	public static double ELEVATOR_MAX_HEIGHT = 89;
	private static double IN_POSITION_BAND = 3;
	public static double outtakeSpeed = -1;// * Robot.joystickSlider + 0.1; // For adjusting Speed lower cased because
											// is no
											// longer constant

	public static double CUBE_FAST_OUTTAKE = -.65;
	public static double CUBE_SLOW_OUTTAKE = -.25;
	public static double INTAKE_SPEED = .5;

	// public double holdPositionEncoderCounts;
	public double holdPositionInches;
	public static double ELEVATOR_PICKUP_POSITION_INCHES = .5;
	public static double ELEVATOR_EXCHANGE_POSITION_INCHES = 3.5;
	public static double ELEVATOR_AUTO_FIRST_POSITION_INCHES = 4.5;
	public static double ELEVATOR_TRAVEL_POSITION_INCHES = 10;
	public static double ELEVATOR_SWITCH_POSITION_INCHES = 26;
	public static double ELEVATOR_SWITCH_HIGH_POSITION_INCHES = 34;
	public static double ELEVATOR_SCALE_LOW_POSITION_INCHES = 65;
	public static double ELEVATOR_SCALE_POSITION_INCHES = 85;
	public static double ELEVATOR_PORTAL_POSITION_INCHES = 17;
	public static double ELEVATOR_POSITION_RATE = 40;// in per sec
	// (in/sec) * enc Counts/in = enc counts / sec then divide by 10 for 100ms
	public static double IN_PER_SEC_TO_ENC_CTS_PER_100MS = ENCODER_COUNTS_PER_INCH / 10;
	// public static int ELEVATOR_ACCEL_RATE = 50;

	public static String[] elevatorPrefsNames = new String[8];
	public static double[] elevatorPrefsDefaults = new double[8];

	public boolean brakeState;

	public boolean elevatorTooHigh;
	public boolean elevatorTooLow;
	public boolean moveIsUp;
	public boolean moveIsDown;

	public double elevatorTargetPosition;
	private boolean switchWasSeen;
	// public boolean elevatorMotionDown;
	public boolean cubePickedUp;
	public double lastHoldPositionInches;
	private int elevatorHiCurrent;

	public enum intakeSide {
		left, right
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// setDefaultCommand(new RunElevatorFromGamepad());
		setDefaultCommand(new HoldElevatorPositionMotionMagic());
	}

	public void initPrefs() {
		elevatorPrefsNames[0] = "ElevatorMMKf";
		elevatorPrefsDefaults[0] = .75;
		elevatorPrefsNames[1] = "ElevatorMMKp";
		elevatorPrefsDefaults[1] = 0.2;
		elevatorPrefsNames[2] = "ElevatorMMKi";
		elevatorPrefsDefaults[2] = 0;
		elevatorPrefsNames[3] = "ElevatorMMKd";
		elevatorPrefsDefaults[3] = 0.;
		// down values no longer used - same as up
		elevatorPrefsNames[4] = "ElevatorDownMMKf";
		elevatorPrefsDefaults[4] = .4;
		elevatorPrefsNames[5] = "ElevatorDownMMKp";
		elevatorPrefsDefaults[5] = 0.09;
		elevatorPrefsNames[6] = "ElevatorDownMMKi";
		elevatorPrefsDefaults[6] = 0;
		elevatorPrefsNames[7] = "ElevatorDownMMKd";
		elevatorPrefsDefaults[7] = 0;
	}

	public void intakeWheelsTurn(double speed) {
		RobotMap.intakeLeftMotor.set(ControlMode.PercentOutput, speed);
		RobotMap.intakeRightMotor.set(ControlMode.PercentOutput, speed);
	}

	public void turnOneIntakeSide(double speed, intakeSide side) {
		switch (side) {
		case left:
			RobotMap.intakeLeftMotor.set(ControlMode.PercentOutput, speed);
			break;
		case right:
			RobotMap.intakeRightMotor.set(ControlMode.PercentOutput, speed);
			break;
		default:
			break;
		}
	}

	public void openIntakeArms() {
		RobotMap.intakeArmsOpen.set(DoubleSolenoid.Value.kReverse);
	}

	public void closeIntakeArms() {
		RobotMap.intakeArmsOpen.set(DoubleSolenoid.Value.kForward);

	}

	public int getElevatorEncoderPosition() {
		return RobotMap.elevatorMotor.getSelectedSensorPosition(0);
	}

	public double getElevatorPositionInches() {
		return Math.round(RobotMap.elevatorMotor.getSelectedSensorPosition(0) / ENCODER_COUNTS_PER_INCH);
	}

	public int getElevatorEncoderSpeedCountsPer100mS() {
		return RobotMap.elevatorMotor.getSelectedSensorVelocity(0);
	}

	public double getElevatorSpeedInchesPerSecond() {
		return getElevatorEncoderSpeedCountsPer100mS() / IN_PER_SEC_TO_ENC_CTS_PER_100MS;
	}

	public boolean getElevatorAboveSwitch() {
		return getElevatorPositionInches() > (ELEVATOR_SWITCH_POSITION_INCHES - 4);
	}

	public boolean getElevatorAboveScale() {
		return getElevatorPositionInches() > (ELEVATOR_SCALE_POSITION_INCHES - 4);
	}

	public void magicMotionElevator(double distance, double speedIPS) {
		// elevator motor 775 Pro with 70:1 gear reduction and a 4096 count encoder
		//
		// Motor data 18000 rpm = 300 rps = 30 revs/100ms
		//
		//
		// measured rate at 100% was 1500
		//
		// Use measured rate not theoretical so 100% Kf would be 1023/1500 =.7
		//
		//
		//
		// For error of 1 inch, to add another 2% of motor
		// output. p-gain .02 x 1023 / (341) = .06
		//
		// start P-gain = .06
		//

		elevatorTargetPosition = distance;

		/*
		 * set acceleration and cruise velocity - see documentation accel is in units of
		 * enc cts per 100ms per second so to accelerate in 1/2 second, use velocity x 2
		 */

		int cruiseVelocity = (int) (speedIPS * IN_PER_SEC_TO_ENC_CTS_PER_100MS);

		int acceleration;

		if (Robot.cubeHandler.moveIsDown) {
			cruiseVelocity = (cruiseVelocity * 3) / 4;
			acceleration = cruiseVelocity;// 1 second to soften bottom hit
		} else
			acceleration = cruiseVelocity;// 1/2 second

		RobotMap.elevatorMotor.configMotionCruiseVelocity(cruiseVelocity, 0);
		RobotMap.elevatorMotor.configMotionAcceleration(acceleration, 0);
		RobotMap.elevatorMotor.set(ControlMode.MotionMagic, distance * CubeHandler.ENCODER_COUNTS_PER_INCH);
	}

	public boolean inPosition() {
		return Math.abs(elevatorTargetPosition - getElevatorPositionInches()) < IN_POSITION_BAND;
	}

	public void runElevatorMotor(double speed) {
		if (elevatorTooLow && speed < 0)
			speed = 0;
		if (elevatorTooHigh && speed > 0)
			speed = 0;
		SmartDashboard.putNumber("speed", speed);
		RobotMap.elevatorMotor.set(ControlMode.PercentOutput, speed);
	}

	public void resetElevatorPosition() {
		RobotMap.elevatorMotor.setSelectedSensorPosition(0, 0, 0);
		holdPositionInches = getElevatorPositionInches();
	}

	public void updateStatus() {
		// check for elevator unable to reach position for 250 * 20ms = 5 sec
		if (RobotMap.elevatorMotor.getOutputCurrent() > 6)
			elevatorHiCurrent++;
		else
			elevatorHiCurrent = 0;
		if (elevatorHiCurrent > 250)
			holdPositionInches = getElevatorPositionInches();

		elevatorTooLow = getElevatorPositionInches() <= ELEVATOR_MIN_HEIGHT;
		elevatorTooHigh = getElevatorPositionInches() >= ELEVATOR_MAX_HEIGHT;

		SD.putN1("Elevator Amps", RobotMap.elevatorMotor.getOutputCurrent());
		SD.putN1("Elevator Inches", getElevatorPositionInches());

		SmartDashboard.putBoolean("Elevator Too Low", elevatorTooLow);
		SmartDashboard.putBoolean("Elevator Too High", elevatorTooHigh);
		SmartDashboard.putBoolean("Elev In Pos", inPosition());
		SmartDashboard.putNumber("Elevator Encoder", RobotMap.elevatorMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Elevator EncCtsPer100ms", RobotMap.elevatorMotor.getSelectedSensorVelocity(0));
		SD.putN1("Elevator Target", elevatorTargetPosition);
		SD.putN1("Elevator Hold", holdPositionInches);
		SD.putN1("Elevator Last Hold", lastHoldPositionInches);
		SD.putN1("Elevator Pct V", RobotMap.elevatorMotor.getMotorOutputPercent());
		SD.putN1("Elevator Speed IPS", getElevatorSpeedInchesPerSecond());
		SD.putN1("Intake Amps Left", RobotMap.intakeLeftMotor.getOutputCurrent());
		SD.putN1("Intake Amps Right", RobotMap.intakeRightMotor.getOutputCurrent());
		SD.putN1("ElI", RobotMap.elevatorMotor.getIntegralAccumulator(0));
		SD.putN1("El Talon Temp", RobotMap.elevatorMotor.getTemperature());
		SmartDashboard.putBoolean("Elevator Switch", RobotMap.elevatorSwitch.get());
		SmartDashboard.putBoolean("Switch Was Seen", switchWasSeen);

		if (!RobotMap.elevatorSwitch.get() && !switchWasSeen) {
			resetElevatorPosition();
			holdPositionInches = getElevatorPositionInches();
			switchWasSeen = true;
		}
		if (switchWasSeen)
			switchWasSeen = !RobotMap.elevatorSwitch.get();
	}
}
