package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.commands.LowerWings;
import org.usfirst.frc.team2194.robot.commands.RaiseWings;
import org.usfirst.frc.team2194.robot.commands.ResetScanValues;
import org.usfirst.frc.team2194.robot.commands.Climber.DriveClimber;
import org.usfirst.frc.team2194.robot.commands.Climber.StopClimber;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.CloseIntakeArms;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.OpenIntakeArms;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ResetElevatorEncoder;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.RunElevatorFromGamepad;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.SpinCube;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.TurnIntakeWheels;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.VariableOuttake;
import org.usfirst.frc.team2194.robot.commands.Motion.DoRobotMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.DoRobotOrient;
import org.usfirst.frc.team2194.robot.commands.Motion.DoRobotPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.DoRobotTurnToVision;
import org.usfirst.frc.team2194.robot.commands.Motion.DoRobotVisionPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.DoTestReverseTrajectory;
import org.usfirst.frc.team2194.robot.commands.Motion.DoTestTrajectory;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetEncoders;
import org.usfirst.frc.team2194.robot.commands.Motion.ResetGyro;
import org.usfirst.frc.team2194.robot.commands.Motion.ToggleCloseDriveSpeedLoop;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	public Gamepad gamepad;
	public Joystick joystick1;

	public JoystickButton hiGear;

	public JoystickButton spinCube;

	public JoystickButton endCommand;

	public JoystickButton climbForward;
	public JoystickButton climbBackward;
	public JoystickButton stopClimb;

	public JoystickButton intakeCube;
	public JoystickButton ejectCube;

	public JoystickButton openIntakeArms;
	public JoystickButton closeIntakeArms;
	public JoystickButton outtakeCube;

	public JoystickButton startIntakeWheels;
	public JoystickButton reverseIntakeWheels;
	public JoystickButton stopIntakeWheels;

	public JoystickButton elevatorToBottomPosition;
	public JoystickButton elevatorToTravelPosition;
	public JoystickButton elevatorToSwitchPosition;
	public JoystickButton elevatorToPortalPosition;
	public JoystickButton elevatorToScalePosition;
	public JoystickButton elevatorToScaleLowPosition;
	public JoystickButton elevatorToExchange;

	public JoystickButton jogElevator;
	public JoystickButton variableOut;
	
	public JoystickButton lowerWings;
	public JoystickButton raiseWings;

	public OI() {
		gamepad = new Gamepad(0);
		joystick1 = new Joystick(1);

		intakeCube = new JoystickButton(joystick1, 1);
		// intakeCube.whenPressed(new IntakeCube());
		intakeCube.whileHeld(new TurnIntakeWheels(CubeHandler.INTAKE_SPEED));
		intakeCube.whenReleased(new TurnIntakeWheels(0));
		
		variableOut = new JoystickButton(joystick1, 2);
		variableOut.whileHeld(new VariableOuttake());
		variableOut.whenReleased(new TurnIntakeWheels(0));
		
		closeIntakeArms = new JoystickButton(joystick1, 3);
		closeIntakeArms.whenPressed(new CloseIntakeArms());
		
		spinCube = new JoystickButton(joystick1, 4);
		spinCube.whileHeld(new SpinCube(true));
		spinCube.whenReleased(new TurnIntakeWheels(0));
		
		
		openIntakeArms = new JoystickButton(joystick1, 5);
		openIntakeArms.whenPressed(new OpenIntakeArms());
		
		
		climbBackward = new JoystickButton(joystick1, 8);
		climbBackward.whileHeld(new DriveClimber(-.9));
		climbBackward.whenReleased(new StopClimber());
		
		climbForward = new JoystickButton(joystick1, 7);
		climbForward.whileHeld(new DriveClimber(.9));
		climbForward.whenReleased(new StopClimber());
		
		lowerWings = new JoystickButton(joystick1, 9);
		lowerWings.whenPressed(new LowerWings());
		
		raiseWings = new JoystickButton(joystick1, 10);
		raiseWings.whenPressed(new RaiseWings());

		

		

		

		

		elevatorToBottomPosition = gamepad.getButtonA();
		elevatorToBottomPosition.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

		elevatorToTravelPosition = gamepad.getButtonB();
		elevatorToTravelPosition.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_TRAVEL_POSITION_INCHES));
		
		elevatorToScalePosition = gamepad.getButtonY();
		elevatorToScalePosition.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SCALE_POSITION_INCHES));

		elevatorToScaleLowPosition = gamepad.getButtonX();
		elevatorToScaleLowPosition
				.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SCALE_LOW_POSITION_INCHES));

		elevatorToSwitchPosition = gamepad.getLeftShoulder();
		elevatorToSwitchPosition.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_SWITCH_POSITION_INCHES));
		
		elevatorToPortalPosition = gamepad.getRightShoulder();
		elevatorToPortalPosition
		.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PORTAL_POSITION_INCHES));
		
		stopIntakeWheels = gamepad.getLeftTriggerClick();
		//stopIntakeWheels.whenPressed(new TurnIntakeWheels(0));
		stopIntakeWheels.whileHeld(new DriveClimber(-.9));
		stopIntakeWheels.whenReleased(new StopClimber());
		elevatorToExchange = gamepad.getRightTriggerClick();
		elevatorToExchange.whenPressed(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_EXCHANGE_POSITION_INCHES));

		jogElevator = gamepad.getStartButton();
		jogElevator.whileHeld(new RunElevatorFromGamepad());

		

		SmartDashboard.putData("Reset Encoders", new ResetEncoders());
		SmartDashboard.putData("Reset Gyro", new ResetGyro());
		SmartDashboard.putData("Toggle Closed Loop", new ToggleCloseDriveSpeedLoop());

		SmartDashboard.putData("Reset Values", new ResetScanValues());

		SmartDashboard.putData("Position Robot", new DoRobotPosition());
		SmartDashboard.putData("Orient Robot", new DoRobotOrient());
		SmartDashboard.putData("Test Trajectory", new DoTestTrajectory());
		SmartDashboard.putData("Test Rev Trajectory", new DoTestReverseTrajectory());
		SmartDashboard.putData("Magic Motion Robot", new DoRobotMagicMotion());
		SmartDashboard.putData("Vision Motion Robot", new DoRobotVisionPosition());
		SmartDashboard.putData("Turn To Vision Robot", new DoRobotTurnToVision());
		SmartDashboard.putData("Reset Elevator Position", new ResetElevatorEncoder());

	}

	public Joystick getgamepad() {
		return gamepad;
	}

	public Joystick getJoystick1() {
		return joystick1;
	}
}
