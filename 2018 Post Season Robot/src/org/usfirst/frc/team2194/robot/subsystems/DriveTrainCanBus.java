package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.ReverseDistanceFollower;
import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.SD;
import org.usfirst.frc.team2194.robot.commands.Motion.RunFromGamepadCanBus;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.followers.DistanceFollower;

public class DriveTrainCanBus extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private TalonSRX leftMotorA = RobotMap.driveLeftMotorA;
	private TalonSRX rightMotorA = RobotMap.driveRightMotorA;
	private TalonSRX leftMotorB = RobotMap.driveLeftMotorB;
	private TalonSRX rightMotorB = RobotMap.driveRightMotorB;
	private TalonSRX leftMotorC = RobotMap.driveLeftMotorC;
	private TalonSRX rightMotorC = RobotMap.driveRightMotorC;
	public double driveStraightAngle;
	public double leftPositionTargetFt;
	public double rightPositionTargetFt;

	public double positionStraightKp;
	public double driveStraightKp;
	public double visionStraightKp;

	public double JOYSTICK_TURN_CONSTANT = 1.6;

	public DistanceFollower leftDf = new DistanceFollower();
	public DistanceFollower rightDf = new DistanceFollower();

	public ReverseDistanceFollower revLeftDf = new ReverseDistanceFollower();
	public ReverseDistanceFollower revRightDf = new ReverseDistanceFollower();

	// ***************
	// * CONSTANTS *
	// ***************
	public static double joystickYDeadband = .1;
	public static double joystickXDeadband = .1;
	// public static double encoderCountsPerRev = 400;
	// public static double inchesPerEncoderRev = 4*Math.PI;
	public static double DRIVE_ENCODER_COUNTS_PER_INCH = 403.;// 32.515;
	// (ft *12/sec) = (in/sec) * enc Counts/in = enc counts / sec then divide by 10
	// for 100ms
	public double FT_PER_SEC_TO_ENC_CTS_PER_100MS = DRIVE_ENCODER_COUNTS_PER_INCH * 1.2;

	public static double MAX_ROBOT_FT_PER_SEC = 10;

	public double MAX_ENC_CTS_PER_100MS = MAX_ROBOT_FT_PER_SEC * FT_PER_SEC_TO_ENC_CTS_PER_100MS;

	public static double FT_PER_SEC_TO_PCT_OUT = 1 / MAX_ROBOT_FT_PER_SEC;

	public static double MINIMUM_START_PCT = .1;// pct needed to get robot moving;

	public double IN_POSITION_BANDWIDTH = .075;
	public double MM_IN_POSITION_BANDWIDTH = .4;
	public double CLOSE_POSITION_BANDWIDTH = .4;

	private int MAGIC_MOTION_ACCEL_FPSPS = 10;
	public double ROBOT_WIDTH = 2.84;
	public double ROBOT_LENGTH = 3.25;
	public double WHEELBASE_WIDTH = 2.17;// ft

	public double MM_FT_PER_DEGREE = Math.PI * WHEELBASE_WIDTH / 360;

	public boolean leftSideStopped;
	public boolean rightSideStopped;
	public boolean runStalledDetect;

	public static String[] drivePrefsNames = new String[25];
	public static double[] drivePrefsDefaults = new double[25];

	public enum driveSide {
		left, right, both
	}

	// order is Kp, Kd, Ka and Kturn
	public static double[] LSW_L = { .4, 0, 0.02, 1 };
	public static double[] LSW_C = { .4, 0, 0.02, 1 };
	public static double[] RSW_C = { .4, 0, 0.02, 1 };
	public static double[] RSW_R = { .4, 0, 0.02, 1 };

	public static double[] LSW_L2 = { .8, 1.2, 0, .6 };
	public static double[] LSW_C2 = { .5, 0, 0, .8 };
	public static double[] LSW_R = { .8, 0, 0, .1 };
	public static double[] LSW_R2 = { .8, 0, 0, .1 };

	// order is Kp, Kd, Ka and Kturn

	public static double[] RSW_C2 = { .8, .5, .06, .8 };
	public static double[] RSW_R2 = { .8, 0, 0, .1 };
	public static double[] RSW_L = { .8, .5, .06, .8 };
	public static double[] RSW_L2 = { .8, .5, .06, .8 };

	// order is Kp, Kd, Ka and Kturn
	public static double[] LSC_L = { .8, 0, 0, .8 };
	public static double[] RSC_L = { 3, 4.0, 0.2, .5 };
	public static double[] RSC_R = { .8, 0, 0, .1 };
	public static double[] LSC_R = { 3, 4.0, 0.2, .5 };
	public static double[] Test = { .8, 0, 0, .1 };
	public static double[] RevTest = { .8, 0, 0, .1 };

	private int test;
	public double leftSpeedForDebug;
	public double rightSpeedForDebug;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new RunFromGamepadCanBus());

	}

	public void initPrefs() {
		drivePrefsNames[0] = "Kp";
		drivePrefsDefaults[0] = .09;

		drivePrefsNames[1] = "Ki";
		drivePrefsDefaults[1] = .0;

		drivePrefsNames[2] = "Izone";
		drivePrefsDefaults[2] = 2 * DRIVE_ENCODER_COUNTS_PER_INCH;

		drivePrefsNames[3] = "MMKf";
		drivePrefsDefaults[3] = .21;

		drivePrefsNames[4] = "MMKp";
		drivePrefsDefaults[4] = 1.9;

		drivePrefsNames[5] = "MMKi";
		drivePrefsDefaults[5] = 0;

		drivePrefsNames[6] = "MMKd";
		drivePrefsDefaults[6] = 0;

		drivePrefsNames[7] = "notUsed";
		drivePrefsDefaults[7] = .21;

		drivePrefsNames[8] = "DriveStraightKp";
		drivePrefsDefaults[8] = 0.05;

		drivePrefsNames[9] = "PositionStraightKp";
		drivePrefsDefaults[9] = 0.07;

		drivePrefsNames[10] = "RobotRotateKp";
		drivePrefsDefaults[10] = .013;

		drivePrefsNames[11] = "RobotRotateKi";
		drivePrefsDefaults[11] = 0.0000;

		drivePrefsNames[12] = "RobotRotateIzone";
		drivePrefsDefaults[12] = 10;

		drivePrefsNames[13] = "VisionStraightKp";
		drivePrefsDefaults[13] = .01;

		drivePrefsNames[14] = "PathP";
		drivePrefsDefaults[14] = .4;

		drivePrefsNames[15] = "PathD";
		drivePrefsDefaults[15] = .0;

		drivePrefsNames[16] = "PathA";
		drivePrefsDefaults[16] = .0;

		drivePrefsNames[17] = "PathTurn ";
		drivePrefsDefaults[17] = .1;

		drivePrefsNames[18] = "PathP2";
		drivePrefsDefaults[18] = .4;

		drivePrefsNames[19] = "PathD2";
		drivePrefsDefaults[19] = .0;

		drivePrefsNames[20] = "PathA2";
		drivePrefsDefaults[20] = .0;

		drivePrefsNames[21] = "PathTurn2 ";
		drivePrefsDefaults[21] = .1;

		drivePrefsNames[22] = "RobotRotateKd";
		drivePrefsDefaults[22] = .03;

		drivePrefsNames[23] = "MMIzone";
		drivePrefsDefaults[23] = 0;

		drivePrefsNames[24] = "Kd";
		drivePrefsDefaults[24] = .0;
	}

	public void resetEncoders() {
		leftMotorA.setSelectedSensorPosition(0, 0, 0);
		rightMotorA.setSelectedSensorPosition(0, 0, 0);
	}

	public double getLeftInches() {
		return leftMotorA.getSelectedSensorPosition(0) / DRIVE_ENCODER_COUNTS_PER_INCH;
	}

	public int getLeftEncoder() {
		return leftMotorA.getSelectedSensorPosition(0);
	}

	public double getLeftFeetPerSecond() {
		return ((leftMotorA.getSelectedSensorVelocity(0)) / FT_PER_SEC_TO_ENC_CTS_PER_100MS);
	}

	public int getLeftSensorVelocity() {
		return leftMotorA.getSelectedSensorVelocity(0);
	}

	public int getRightEncoder() {
		return rightMotorA.getSelectedSensorPosition(0);
	}

	public double getRightInches() {
		return rightMotorA.getSelectedSensorPosition(0) / DRIVE_ENCODER_COUNTS_PER_INCH;
	}

	public double getRightFeetPerSecond() {
		return ((rightMotorA.getSelectedSensorVelocity(0)) / FT_PER_SEC_TO_ENC_CTS_PER_100MS);
	}

	public int getRightSensorVelocity() {
		return rightMotorA.getSelectedSensorVelocity(0);
	}

	public double getLeftFeet() {
		return getLeftInches() / 12;
	}

	public double getRightFeet() {
		return getRightInches() / 12;
	}

	public double getRobotPositionFeet() {
		return (getLeftFeet() + getRightFeet()) / 2;
	}

	public double getJoystickY() {
		double joystickActual = Robot.oi.joystick1.getY();
		double work = (Math.abs(joystickActual) - joystickYDeadband) / (1 - joystickYDeadband);
		if (Robot.oi.joystick1.getY() < 0)
			work = -work;
		if (Math.abs(joystickActual) < joystickYDeadband)
			work = 0;
		return work;
	}

	public double getJoystickTwist() {
		double joystickActual = Robot.oi.joystick1.getTwist();
		double work = (Math.abs(joystickActual) - joystickYDeadband) / (1 - joystickYDeadband);
		if (Robot.oi.joystick1.getTwist() < 0)
			work = -work;
		if (Math.abs(joystickActual) < joystickYDeadband)
			work = 0;

		return work / JOYSTICK_TURN_CONSTANT;
	}

	public double getJoystickX() {
		double joystickActual = Robot.oi.joystick1.getX();
		double work = (Math.abs(joystickActual) - joystickXDeadband) / (1 - joystickXDeadband);
		if (Robot.oi.joystick1.getX() < 0)
			work = -work;
		if (Math.abs(joystickActual) < joystickXDeadband)
			work = 0;
		return work / JOYSTICK_TURN_CONSTANT;
	}

	public void leftDriveOut(double speed) {
		leftSpeedForDebug = speed;
		if (!Robot.closeDriveSpeedLoop)
			leftMotorA.set(ControlMode.PercentOutput, speed);
		else
			leftMotorA.set(ControlMode.Velocity, speed * MAX_ENC_CTS_PER_100MS);
	}

	public void rightDriveOut(double speed) {
		rightSpeedForDebug = speed;
		if (!Robot.closeDriveSpeedLoop)
			rightMotorA.set(ControlMode.PercentOutput, speed);
		else
			rightMotorA.set(ControlMode.Velocity, speed * MAX_ENC_CTS_PER_100MS);
	}

	public void tankDrive(double leftValue, double rightValue, double comp) {
		leftDriveOut(leftValue - comp);
		rightDriveOut(rightValue + comp);
	}

	public void arcadeDrive(double throttleValue, double turnValue, double comp) {
		leftDriveOut(throttleValue + turnValue - comp);
		rightDriveOut(throttleValue - turnValue + comp);
	}

	/*
	 * Slot 0 is used for velocity gains for 100% speed feed forward F-gain =
	 * ([Percent Output] x 1023) / [Velocity enc cts per 100ms] max enc cts per
	 * 100ms = (10 * 12 * 403)/10 = 12 * 403 = 4840 so Kf = (1 * 1023) / 4840 = .211
	 * Check Kf 50% speed. Command = 2420. Output = .211 * 2420 = Kp is in output
	 * per unit of error Start with Kp at .15
	 */
	public void setVelocityGains() {
		RobotMap.driveLeftMotorA.selectProfileSlot(0, 0);
		RobotMap.driveRightMotorA.selectProfileSlot(0, 0);

		RobotMap.driveLeftMotorA.config_kF(0, .21, 0);
		RobotMap.driveRightMotorA.config_kF(0, .21, 0);

		RobotMap.driveLeftMotorA.config_kP(0, .15, 0);
		RobotMap.driveRightMotorA.config_kP(0, .15, 0);
	}

	public void setBrakeMode(boolean brakeOn) {

		if (brakeOn) {
			RobotMap.driveLeftMotorA.setNeutralMode(NeutralMode.Brake);
			RobotMap.driveLeftMotorB.setNeutralMode(NeutralMode.Brake);
			RobotMap.driveLeftMotorC.setNeutralMode(NeutralMode.Brake);
			RobotMap.driveRightMotorA.setNeutralMode(NeutralMode.Brake);
			RobotMap.driveRightMotorB.setNeutralMode(NeutralMode.Brake);
			RobotMap.driveRightMotorC.setNeutralMode(NeutralMode.Brake);
		}

		else {
			RobotMap.driveLeftMotorA.setNeutralMode(NeutralMode.Coast);
			RobotMap.driveLeftMotorB.setNeutralMode(NeutralMode.Coast);
			RobotMap.driveLeftMotorC.setNeutralMode(NeutralMode.Coast);
			RobotMap.driveRightMotorA.setNeutralMode(NeutralMode.Coast);
			RobotMap.driveRightMotorB.setNeutralMode(NeutralMode.Coast);
			RobotMap.driveRightMotorC.setNeutralMode(NeutralMode.Coast);
		}
	}

	public void configDrivePeakout(double feetPerSecond, driveSide side) {
		if (side != driveSide.right) {
			leftMotorA.configPeakOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			leftMotorA.configPeakOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			leftMotorB.configPeakOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			leftMotorB.configPeakOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			leftMotorC.configPeakOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			leftMotorC.configPeakOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
		}
		if (side != driveSide.left) {
			rightMotorA.configPeakOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			rightMotorA.configPeakOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			rightMotorB.configPeakOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			rightMotorB.configPeakOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			rightMotorC.configPeakOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			rightMotorC.configPeakOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
		}
	}

	public void configDriveNominalOut(double feetPerSecond, driveSide side) {
		if (side != driveSide.right) {
			leftMotorA.configNominalOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			leftMotorA.configNominalOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
		}
		if (side != driveSide.left) {
			rightMotorA.configNominalOutputForward(feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
			rightMotorA.configNominalOutputReverse(-feetPerSecond * FT_PER_SEC_TO_PCT_OUT, 0);
		}
	}

	public void configClosedLoopAcceleration(double seconds) {
		leftMotorA.configClosedloopRamp(seconds, 0);
		rightMotorA.configClosedloopRamp(seconds, 0);
	}

	public void configOpenLoopAcceleration(double seconds) {
		leftMotorA.configOpenloopRamp(seconds, 0);
		rightMotorA.configOpenloopRamp(seconds, 0);
		leftMotorB.configOpenloopRamp(seconds, 0);
		rightMotorB.configOpenloopRamp(seconds, 0);
		leftMotorC.configOpenloopRamp(seconds, 0);
		rightMotorC.configOpenloopRamp(seconds, 0);
	}

	public void setPosition(double positionFt, driveSide side, double feetPerSecond) {

		RobotMap.driveLeftMotorA.selectProfileSlot(1, 0);
		RobotMap.driveRightMotorA.selectProfileSlot(1, 0);

		RobotMap.driveLeftMotorA.config_kP(1, Robot.prefs.getDouble("Kp", drivePrefsDefaults[0]), 0);
		RobotMap.driveRightMotorA.config_kP(1, Robot.prefs.getDouble("Kp", drivePrefsDefaults[0]), 0);

		RobotMap.driveLeftMotorA.config_kI(1, Robot.prefs.getDouble("Ki", drivePrefsDefaults[1]), 0);
		RobotMap.driveRightMotorA.config_kI(1, Robot.prefs.getDouble("Ki", drivePrefsDefaults[1]), 0);

		RobotMap.driveLeftMotorA.config_kD(1, Robot.prefs.getDouble("Kd", drivePrefsDefaults[24]), 0);
		RobotMap.driveRightMotorA.config_kD(1, Robot.prefs.getDouble("Kd", drivePrefsDefaults[24]), 0);

		RobotMap.driveLeftMotorA.config_IntegralZone(1, Robot.prefs.getInt("Izone", (int) drivePrefsDefaults[2]), 0);
		RobotMap.driveRightMotorA.config_IntegralZone(1, Robot.prefs.getInt("Izone", (int) drivePrefsDefaults[2]), 0);

		leftPositionTargetFt = positionFt;
		rightPositionTargetFt = positionFt;
		configDriveNominalOut(0, side);

		switch (side) {
		case left:
			leftMotorA.set(ControlMode.Position, positionFt * 12 * DRIVE_ENCODER_COUNTS_PER_INCH);
			configDrivePeakout(feetPerSecond, side);
			break;
		case right:
			rightMotorA.set(ControlMode.Position, (positionFt * 12 * DRIVE_ENCODER_COUNTS_PER_INCH));
			configDrivePeakout(feetPerSecond, side);
			break;
		case both:
			leftMotorA.set(ControlMode.Position, positionFt * 12 * DRIVE_ENCODER_COUNTS_PER_INCH);
			rightMotorA.set(ControlMode.Position, positionFt * 12 * DRIVE_ENCODER_COUNTS_PER_INCH);
			configDrivePeakout(feetPerSecond, side);
			break;

		}
	}

	public void magicMotionDrive(double distance, double speedFPS, driveSide side) {

		/*
		 * set acceleration and cruise velocity - see documentation
		 * 
		 * accel is in encCtsper100ms per sec addSequential(new ResetEncoders());
		 * 
		 * 
		 * for 1/2 second accel decel
		 * 
		 * accel = cruise velocity x 2
		 * 
		 */

		int cruiseVelocity = (int) (speedFPS * FT_PER_SEC_TO_ENC_CTS_PER_100MS);
		int acceleration = cruiseVelocity * 2;

		if (side != driveSide.right) {
			RobotMap.driveLeftMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
			RobotMap.driveLeftMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
			RobotMap.driveLeftMotorA.selectProfileSlot(2, 0);
			RobotMap.driveLeftMotorA.config_kF(2, Robot.prefs.getDouble("MMKf", drivePrefsDefaults[3]), 0);
			RobotMap.driveLeftMotorA.config_kP(2, Robot.prefs.getDouble("MMKp", drivePrefsDefaults[4]), 0);
			RobotMap.driveLeftMotorA.config_kI(2, Robot.prefs.getDouble("MMKi", drivePrefsDefaults[5]), 0);
			RobotMap.driveLeftMotorA.config_kD(2, Robot.prefs.getDouble("MMKd", drivePrefsDefaults[6]), 0);
			RobotMap.driveLeftMotorA.config_IntegralZone(2,
					(int) Robot.prefs.getDouble("MMIzone", drivePrefsDefaults[23]), 0);
			RobotMap.driveLeftMotorA.configOpenloopRamp(0, 0);
			RobotMap.driveLeftMotorA.configClosedloopRamp(0, 0);
			RobotMap.driveLeftMotorA.configPeakOutputForward(1, 0);
			RobotMap.driveLeftMotorA.configPeakOutputReverse(-1, 0);
			RobotMap.driveLeftMotorA.configMotionCruiseVelocity(cruiseVelocity, 0);
			RobotMap.driveLeftMotorA.configMotionAcceleration(acceleration, 0);
			RobotMap.driveLeftMotorA.set(ControlMode.MotionMagic, distance * 12 * DRIVE_ENCODER_COUNTS_PER_INCH);

		}
		if (side != driveSide.left) {
			RobotMap.driveRightMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
			RobotMap.driveRightMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
			RobotMap.driveRightMotorA.selectProfileSlot(2, 0);
			RobotMap.driveRightMotorA.config_kF(2, Robot.prefs.getDouble("MMKf", drivePrefsDefaults[3]), 0);
			RobotMap.driveRightMotorA.config_kP(2, Robot.prefs.getDouble("MMKp", drivePrefsDefaults[4]), 0);
			RobotMap.driveRightMotorA.config_kI(2, Robot.prefs.getDouble("MMKi", drivePrefsDefaults[5]), 0);
			RobotMap.driveRightMotorA.config_kD(2, Robot.prefs.getDouble("MM Kd", drivePrefsDefaults[6]), 0);
			RobotMap.driveRightMotorA.config_IntegralZone(2,
					(int) Robot.prefs.getDouble("MMIzone", drivePrefsDefaults[23]), 0);
			RobotMap.driveRightMotorA.configOpenloopRamp(0, 0);
			RobotMap.driveRightMotorA.configClosedloopRamp(0, 0);
			RobotMap.driveRightMotorA.configPeakOutputForward(1, 0);
			RobotMap.driveLeftMotorA.configPeakOutputReverse(-1, 0);
			RobotMap.driveRightMotorA.configMotionCruiseVelocity(cruiseVelocity, 0);
			RobotMap.driveRightMotorA.configMotionAcceleration(acceleration, 0);
			RobotMap.driveRightMotorA.set(ControlMode.MotionMagic, distance * 12 * DRIVE_ENCODER_COUNTS_PER_INCH);

		}
	}

	public boolean leftSideInPosition() {
		return Math.abs(leftPositionTargetFt - getLeftFeet()) < IN_POSITION_BANDWIDTH;
	}

	public boolean rightSideInPosition() {
		return Math.abs(rightPositionTargetFt - getRightFeet()) < IN_POSITION_BANDWIDTH;
	}

	public boolean closeToPosition() {
		return Math.abs(leftPositionTargetFt - getLeftFeet()) < CLOSE_POSITION_BANDWIDTH
				&& Math.abs(rightPositionTargetFt - getRightFeet()) < CLOSE_POSITION_BANDWIDTH;
	}

	public void setEncoderPosition(driveSide side, double value) {
		if (side != driveSide.right)
			leftMotorA.setSelectedSensorPosition((int) (value * 12 * DRIVE_ENCODER_COUNTS_PER_INCH), 0, 0);
		if (side != driveSide.left)
			rightMotorA.setSelectedSensorPosition((int) (value * 12 * DRIVE_ENCODER_COUNTS_PER_INCH), 0, 0);
	}

	public void stopMotor(driveSide side) {
		if (side != driveSide.right)
			leftMotorA.set(ControlMode.Disabled, 0);
		if (side != driveSide.left)
			rightMotorA.set(ControlMode.Disabled, 0);
	}

	public void motionMagic(double targetPosition, double ftPerSec, double acceleration) {

		/* Set relevant frame periods to be at least as fast as periodic rate */
		configDrivePeakout(8, driveSide.both);

		leftMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		leftMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

		rightMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		rightMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

		leftMotorA.configMotionCruiseVelocity((int) (ftPerSec * FT_PER_SEC_TO_ENC_CTS_PER_100MS), 0);
		leftMotorA.configMotionAcceleration((int) acceleration, 0);

		rightMotorA.configMotionCruiseVelocity((int) (ftPerSec * FT_PER_SEC_TO_ENC_CTS_PER_100MS), 0);
		rightMotorA.configMotionAcceleration((int) acceleration, 0);

		leftMotorA.set(ControlMode.MotionMagic, targetPosition * DRIVE_ENCODER_COUNTS_PER_INCH);
		rightMotorA.set(ControlMode.MotionMagic, targetPosition * DRIVE_ENCODER_COUNTS_PER_INCH);
	}
	
	public void setStatusFramePeriod(int period) {
		RobotMap.driveLeftMotorA.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, 0);
		RobotMap.driveRightMotorA.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, 0);
		RobotMap.driveLeftMotorB.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, 0);
		RobotMap.driveRightMotorB.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, 0);
		RobotMap.driveLeftMotorC.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, 0);
		RobotMap.driveRightMotorC.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, 0);
	}
	
	public void setControlFramePeriod(int period) {
		RobotMap.driveLeftMotorA.setControlFramePeriod(ControlFrame.Control_3_General, period);
		RobotMap.driveRightMotorA.setControlFramePeriod(ControlFrame.Control_3_General, period);
		RobotMap.driveLeftMotorB.setControlFramePeriod(ControlFrame.Control_3_General, period);
		RobotMap.driveRightMotorB.setControlFramePeriod(ControlFrame.Control_3_General, period);
		RobotMap.driveLeftMotorC.setControlFramePeriod(ControlFrame.Control_3_General, period);
		RobotMap.driveRightMotorC.setControlFramePeriod(ControlFrame.Control_3_General, period);
	}

	public void updateStatus() {

		SD.putN2("RightFt", getRightFeet());
		SD.putN2("LeftFt", getLeftFeet());

		SmartDashboard.putBoolean("Speed Mode", Robot.closeDriveSpeedLoop);

		SD.putN1("LeftAmps", leftMotorA.getOutputCurrent());
		SD.putN1("RightAmps", rightMotorA.getOutputCurrent());

		SD.putN1("RightFeetperSec", getRightFeetPerSecond());
		SD.putN1("LeftFeetperSec", getLeftFeetPerSecond());

		SD.putN1("LeftInches", getLeftInches());
		SD.putN1("RightInches", getRightInches());

		SmartDashboard.putBoolean("LinPos", leftSideInPosition());
		SmartDashboard.putBoolean("RinPos", rightSideInPosition());
		SmartDashboard.putBoolean("LisStopped", leftSideStopped);
		SmartDashboard.putBoolean("RisStopped", rightSideStopped);
		// if (Robot.trajectoryRunning) {
		// SmartDashboard.putBoolean("LDfFnshd", leftDf.isFinished());
		// SmartDashboard.putBoolean("RDfFnshd", rightDf.isFinished());
		// }

		SD.putN1("LTarget", leftPositionTargetFt);
		SD.putN1("RTarget", rightPositionTargetFt);

		SD.putN1("LeftPower", RobotMap.driveLeftMotorA.getMotorOutputPercent());
		SD.putN1("RightPower", RobotMap.driveRightMotorA.getMotorOutputPercent());
		SD.putN1("RightEncoderCount", rightMotorA.getSelectedSensorPosition(0));
		SD.putN1("RightVelocity", rightMotorA.getSelectedSensorVelocity(0));
		SD.putN1("LeftEncoderCount", leftMotorA.getSelectedSensorPosition(0));
		SD.putN1("LeftVelocity", leftMotorA.getSelectedSensorVelocity(0));
		SD.putN("TEST", test);
		SD.putN3("LeftSpeedSet", leftSpeedForDebug);
		SD.putN3("RightSpeedSet", rightSpeedForDebug);
	}

}
