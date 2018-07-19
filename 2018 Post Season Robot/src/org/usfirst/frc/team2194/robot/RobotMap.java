package org.usfirst.frc.team2194.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

	public static TalonSRX driveLeftMotorA;
	public static TalonSRX driveLeftMotorB;
	public static TalonSRX driveLeftMotorC;

	public static int driveLeftMotorAID;
	public static int driveRightMotorAID;

	public static TalonSRX driveRightMotorA;
	public static TalonSRX driveRightMotorB;
	public static TalonSRX driveRightMotorC;

	public static TalonSRX climberMotorA;
	public static TalonSRX climberMotorB;
	public static TalonSRX climberMotorC;
	public static TalonSRX climberMotorD;

	public static TalonSRX intakeLeftMotor;
	public static TalonSRX intakeRightMotor;
	public static TalonSRX elevatorMotor;

	public static PowerDistributionPanel pdp;

	public static Compressor compressor;
	public static DoubleSolenoid intakeArmsOpen;
	public static DoubleSolenoid wingShifter;

	public static AnalogInput ultraSoundForward;

	public static DigitalInput testProx;
	public static DigitalInput elevatorSwitch;

	public static void init() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
		driveLeftMotorA = new TalonSRX(3);
		driveLeftMotorAID = driveLeftMotorA.getDeviceID();
		driveLeftMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		driveLeftMotorA.selectProfileSlot(0, 0);
		driveLeftMotorA.configMotionAcceleration(5, 0);
		driveLeftMotorA.configClosedloopRamp(1, 0);
		driveLeftMotorA.setInverted(true);
		driveLeftMotorA.setNeutralMode(NeutralMode.Brake);
		driveLeftMotorA.configVoltageCompSaturation(12, 0);
		driveLeftMotorA.enableVoltageCompensation(false);
		driveLeftMotorA.setSensorPhase(true);
		driveLeftMotorA.configPeakOutputForward(1, 0);
		driveLeftMotorA.configPeakOutputReverse(-1, 0);

		driveLeftMotorB = new TalonSRX(4);
		driveLeftMotorB.set(ControlMode.Follower, driveLeftMotorAID); // set(ControlMode.Follower, masterTalonID);
		driveLeftMotorB.setInverted(true);
		driveLeftMotorB.setNeutralMode(NeutralMode.Brake);
		driveLeftMotorB.configPeakOutputForward(1, 0);
		driveLeftMotorB.configPeakOutputReverse(-1, 0);

		driveLeftMotorC = new TalonSRX(5);
		driveLeftMotorC.set(ControlMode.Follower, driveLeftMotorAID);
		driveLeftMotorC.setInverted(true);
		driveLeftMotorC.setNeutralMode(NeutralMode.Brake);
		driveLeftMotorC.configPeakOutputForward(1, 0);
		driveLeftMotorC.configPeakOutputReverse(-1, 0);

		driveRightMotorA = new TalonSRX(6);
		driveRightMotorAID = driveRightMotorA.getDeviceID();
		driveRightMotorA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		driveRightMotorA.selectProfileSlot(0, 0);
		driveRightMotorA.configMotionAcceleration(5, 0);
		driveRightMotorA.setInverted(false);
		driveRightMotorA.configClosedloopRamp(1, 0);
		driveRightMotorA.setNeutralMode(NeutralMode.Brake);
		driveRightMotorA.configVoltageCompSaturation(12, 0);
		driveRightMotorA.enableVoltageCompensation(false);
		driveRightMotorA.setSensorPhase(true);
		driveRightMotorA.configPeakOutputForward(1, 0);
		driveRightMotorA.configPeakOutputReverse(-1, 0);

		driveRightMotorB = new TalonSRX(7);
		driveRightMotorB.set(ControlMode.Follower, driveRightMotorAID);
		driveRightMotorB.setInverted(false);
		driveRightMotorB.setNeutralMode(NeutralMode.Brake);
		driveRightMotorB.configPeakOutputForward(1, 0);
		driveRightMotorB.configPeakOutputReverse(-1, 0);

		driveRightMotorC = new TalonSRX(8);
		driveRightMotorC.set(ControlMode.Follower, driveRightMotorAID);
		driveRightMotorC.setInverted(false);
		driveRightMotorC.setNeutralMode(NeutralMode.Brake);
		driveRightMotorB.configPeakOutputForward(1, 0);
		driveRightMotorB.configPeakOutputReverse(-1, 0);

		pdp = new PowerDistributionPanel(1);
		compressor = new Compressor(0);

		intakeArmsOpen = new DoubleSolenoid(1, 0);
		intakeArmsOpen.set(DoubleSolenoid.Value.kForward);

		wingShifter = new DoubleSolenoid(2, 3);
		wingShifter.set(DoubleSolenoid.Value.kForward);

		intakeLeftMotor = new TalonSRX(10);
		intakeRightMotor = new TalonSRX(11);
		intakeRightMotor.setInverted(false);

		elevatorMotor = new TalonSRX(12);
		elevatorMotor.setInverted(true);
		elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		elevatorMotor.setSensorPhase(true);
		elevatorMotor.setNeutralMode(NeutralMode.Brake);
		elevatorMotor.configVoltageCompSaturation(12, 0);
		elevatorMotor.enableVoltageCompensation(true);

		climberMotorA = new TalonSRX(13);
		climberMotorB = new TalonSRX(14);
		climberMotorC = new TalonSRX(15);
		climberMotorD = new TalonSRX(16);
		climberMotorC.setInverted(true);
		climberMotorD.setInverted(true);
		climberMotorB.set(ControlMode.Follower, climberMotorA.getDeviceID());
		climberMotorC.set(ControlMode.Follower, climberMotorA.getDeviceID());
		climberMotorD.set(ControlMode.Follower, climberMotorA.getDeviceID());

		ultraSoundForward = new AnalogInput(0);
		ultraSoundForward.setAverageBits(8);

		testProx = new DigitalInput(9);

		elevatorSwitch = new DigitalInput(0);

	}
}
