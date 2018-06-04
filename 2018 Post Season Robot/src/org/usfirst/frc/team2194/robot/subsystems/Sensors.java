package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.SD;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Sensors extends Subsystem {

	public static final double MIN_SCALE_ULTRASOUND_FT = 4;
	public static final double MAX_SCALE_ULTRASOUND_FT = 12;

	public static AHRS imu;
	/*
	 * Sensor Maxbotics MB1013 range is 1 ft to 15 ft.
	 */
	private double mmPerMv = .976;// mm/Mvolt 4.88mV per mm
	public double scaleReadDistance;
	public double scaleMoveDistance;

	public static double SCALE_DISTANCE_CONSTANT = 4.5;// ft

	final static double kCollisionThreshold_DeltaG = 0.5f;
	private boolean collisionDetected = false;

	private double last_world_linear_accel_x;

	private double last_world_linear_accel_y;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// setDefaultCommand(new DoNothing());
	}

	public void initAHRS() {
		try {
			// imu = new AHRS(I2C.Port.kOnboard);

			imu = new AHRS(SPI.Port.kMXP);

			// imu = new AHRS(SerialPort.Port.kUSB1);
			imu.setPIDSourceType(PIDSourceType.kDisplacement);

		} catch (Exception ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}

	}

	public double getGyroYaw() {
		return imu.getYaw();
	}

	public double getGyroAngle() {
		return imu.getAngle();
	}

	public double getGyroRoll() {
		return imu.getRoll();
	}

	public double getGyroPitch() {
		return imu.getPitch();
	}

	public double getGyroError() {
		return imu.getYaw() - Robot.driveTrainCanBus.driveStraightAngle;
	}

	public double getGyroDriveStraightComp() {
		Robot.driveTrainCanBus.driveStraightKp = Robot.prefs.getDouble("DriveStraightKp", .05);
		return Robot.driveTrainCanBus.driveStraightKp * getGyroError();
	}

	public double getGyroPositionStraightComp() {
		Robot.driveTrainCanBus.positionStraightKp = Robot.prefs.getDouble("PositionStraightKp", .05);
		return Robot.driveTrainCanBus.positionStraightKp * getGyroError();
	}

	public void resetGyro() {
		imu.reset();
	}

	public boolean collisionDetect() {

		double curr_world_linear_accel_x = imu.getWorldLinearAccelX();

		double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;

		last_world_linear_accel_x = curr_world_linear_accel_x;

		double curr_world_linear_accel_y = imu.getWorldLinearAccelY();

		double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;

		last_world_linear_accel_y = curr_world_linear_accel_y;

		return (Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) ||

				(Math.abs(currentJerkY) > kCollisionThreshold_DeltaG);
	}

	public double getForwardUltrasoundVoltage() {
		return RobotMap.ultraSoundForward.getAverageVoltage();
	}

	public double getForwardUltrasoundMM() {
		return 1000 * getForwardUltrasoundVoltage() / mmPerMv;
	}

	public double getForwardUltrasoundInches() {
		return getForwardUltrasoundMM() / 25.4;

	}

	public double getForwardUltrasoundFeet() {
		return getForwardUltrasoundInches() / 12;

	}

	public void updateStatus() {
		SD.putN1("Gyro Yaw", getGyroYaw());

		SD.putN1("Gyro Angle", getGyroAngle());
		SD.putN0("Gyro Pitch", getGyroPitch());
		SD.putN0("Gyro Roll", getGyroRoll());
		SmartDashboard.putBoolean("IMU Connected", imu.isConnected());
		SmartDashboard.putBoolean("IMU Calibrating", imu.isCalibrating());
		SD.putN1("Pos Comp", getGyroPositionStraightComp());
		// SD.putN0("US Fwd Inches", getForwardUltrasoundInches());
		// SD.putN1("UltraS Get Forward Out Ft", getForwardUltrasoundFeet());
		// SD.putN3("US Volt Reading", getForwardUltrasoundVoltage());

	}

}
