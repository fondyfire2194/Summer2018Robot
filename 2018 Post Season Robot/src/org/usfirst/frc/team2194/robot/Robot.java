
package org.usfirst.frc.team2194.robot;

import java.io.File;

import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoCrossLineMove;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoLeftSwitchFromLeft;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoRightSwitchFromRight;
import org.usfirst.frc.team2194.robot.commands.Motion.DoTeleopTrajectorySwitch;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveMagicMotion;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToVisionTarget;
import org.usfirst.frc.team2194.robot.commands.Motion.PathfinderReverseTrajectoryUsingNotifier;
import org.usfirst.frc.team2194.robot.commands.Motion.PathfinderTrajectoryUsingNotifier;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.subsystems.AirCompressor;
import org.usfirst.frc.team2194.robot.subsystems.Climber;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;
import org.usfirst.frc.team2194.robot.subsystems.PowerPanel;
import org.usfirst.frc.team2194.robot.subsystems.RobotRotate;
import org.usfirst.frc.team2194.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	CasseroleRIOLoadMonitor loadMon;
	public static RobotDriveMonitor driveMonitor;
	public static SimpleCSVLogger simpleCSVLogger;
	public static DriveTrainCanBus driveTrainCanBus;
	public static Sensors sensors;
	public static RobotRotate robotRotate;
	public static AllCameras allCameras;

	public static CubeHandler cubeHandler;
	public static Climber climber;

	public static AirCompressor airCompressor;
	public static PowerPanel powerPanel;

	// public static OI oi;
	public static OIAlt oi;

	public static Preferences prefs;

	public static BuildTrajectory buildTrajectory;

	public static boolean doMotionOption;
	public static boolean driveStraight;
	public static float yawTarget;
	public static boolean closeDriveSpeedLoop;
	public static boolean orientRunning;
	public static boolean positionRunning;
	public static boolean magicMotionRunning;
	public static boolean visionMotionRunning;

	public static double topEncoderSpeed = 700;
	public static boolean doMMMove;

	// Command firstAutonomousCommand;
	public static Command firstAutonomousCommand;
	private boolean firstAutonomousCommandStarted;
	public static boolean firstAutonomousCommandDone;

	static Command secondAutonomousCommand;
	private boolean secondAutonomousCommandStarted;
	public boolean secondAutonomousCommandsDone;

	Command switchMovesAuxCommand;

	private SendableChooser<Integer> sameSideChooser;
	private SendableChooser<Integer> startPositionChooser;
	private SendableChooser<Integer> oppositeSidePriorityChooser;
	private SendableChooser<Integer> testTrajectoryChooser;

	public boolean onBlueAlliance;
	public boolean onRedAlliance;

	public static boolean trajectoryRunning;
	public static Trajectory activeLeftTrajectory;
	public static Trajectory activeRightTrajectory;
	public static boolean createTrajectoryRunFile = true;;
	public static String chosenFile = "None Chosen";

	private Integer startPosition = 0;
	public static boolean leftStartPosition;;
	public static boolean centerStartPosition;
	public static boolean rightStartPosition;

	public boolean sameSideScaleHasPriority;
	public boolean sameSideSwitchHasPriority;
	public Integer sameSidePriority;

	public static boolean leftSwitchActive;
	public static boolean rightSwitchActive;
	public boolean leftScaleActive;

	public boolean rightScaleActive;
	public boolean otherLeftSwitchActive;
	public boolean otherRightSwitchActive;

	public boolean doLeftSwitchFromLeft;
	public boolean doLeftSwitchFromRight;
	public static boolean doLeftSwitchFromCenter;
	public boolean doRightSwitchFromRight;
	public boolean doRightSwitchFromLeft;
	public static boolean doRightSwitchFromCenter;

	private boolean sameSideAvailable;
	private boolean sameSideSwitchAvailable;
	private boolean sameSideScaleAvailable;

	public boolean doLeftScaleFromLeft;
	public boolean doRightScaleFromRight;
	public boolean doLeftScaleFromRight;
	public boolean doRightScaleFromLeft;
	public boolean doMoveToOppositeSide;

	private Integer oppositeSidePriority;
	public boolean oppositeScaleHasPriority;
	public boolean oppositeSwitchHasPriority;
	public boolean moveToOppositeSide;
	public boolean oppositeDoNotUse;
	public boolean crossLine;
	public boolean doCrossLine;

	private SendableChooser<Double> timeDelayChooser;
	public double timeDelaySeconds;
	private double startTime;
	private double positionTarget = 6.;
	private double positionFPS = 5;
	private double angleTarget = 90;
	private double orientRate = 0.25;
	private int visionTarget;
	public static boolean motionCommandRunning;
	public static boolean motionCommandComplete;

	public static int stoppedValue = 50;
	public static boolean doTeleopOrient;
	public static boolean doTeleopReverseOrient;
	public static boolean doTeleopPosition;
	public static boolean doVelocity;
	public static boolean doTeleopTrajectory;
	public static boolean doTeleopRevTrajectory;
	public static boolean doTeleopMagicMotion;
	public static boolean doTeleopVisionMotion;
	public static boolean doTeleopRotateToVision;
	private Integer testTrajectory;
	private String trajFileName;

	public static boolean cancelGamepad;
	public static boolean isInHighGear;

	public static double joystickSlider;
	public static driveSide stopSide;
	public static int stopSideZeroSegments;

	public long freeMemory;
	public long minFreeMemory = 10000000000000L;
	public long maxFreeMemory = 0L;

	public long currentMs;
	private boolean autoStarted;
	public static boolean isSwitch;
	public static boolean isScale;

	public static double minMs = 10000000000000.;
	public static double maxMs = 0.;

	public enum motionType {
		incremental, absolute
	}

	public static double[] activeTrajectoryGains = { 0, 0, 0, 0 };
	public static double[] activeTrajectoryTwoGains = { 0, 0, 0, 0 };

	private int updateStatusCounter;
	private boolean autonomousSequenceStarted;
	public static boolean teleopAutoRunning;
	private static boolean oppositeSideSwitch;
	private static boolean secondaryTrajectory;
	// Trajectory log data headers
	public static String[] names = { "Step", "LeftCmd", "LeftFt", "RightCmd", "RightFt", "AngleCmd", "Angle",
			"LeftSegVel", "left", "ActLeftVel", "RightSegVel", "right", "ActRightVel", "turn" };
	public static String[] units = { "Number", "FT", "FT", "FT", "FT", "Deg", "Deg", "pct", "pct", "pct", "pct", "pct",
			"pct", "pct" };

	public static String usbFilePath = "/U/TrajCSV/";
	public static boolean createIntakeRunFile = true;
	public static boolean createDriveRunFile;
	public static boolean useVision = false;
	public static double xPosition;
	public static double yPosition;
	public static boolean createElevatorRunFile = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotMap.init();
		DistCon.init();
		// DistCon.updateStatus();
		// loadMon = new CasseroleRIOLoadMonitor();
		driveMonitor = new RobotDriveMonitor();
		prefs = Preferences.getInstance();
		simpleCSVLogger = new SimpleCSVLogger();

		// freeMemory = Runtime.getRuntime().freeMemory();

		powerPanel = new PowerPanel();
		airCompressor = new AirCompressor();

		driveTrainCanBus = new DriveTrainCanBus();
		driveTrainCanBus.initPrefs();
		makePrefs(prefs, DriveTrainCanBus.drivePrefsNames, DriveTrainCanBus.drivePrefsDefaults);
		if (prefs.containsKey("PathTurn "))
			prefs.remove("PathTurn ");
		// if (prefs.containsKey("Robot Rotate Kp"))
		// prefs.remove("Robot Rotate Kp");
		//
		driveTrainCanBus.setLeftBrakeMode(true);
		driveTrainCanBus.setRightBrakeMode(true);

		cubeHandler = new CubeHandler();
		cubeHandler.initPrefs();
		makePrefs(prefs, CubeHandler.elevatorPrefsNames, CubeHandler.elevatorPrefsDefaults);
		cubeHandler.resetElevatorPosition();
		cubeHandler.holdPositionInches = cubeHandler.getElevatorPositionInches();

		sensors = new Sensors();
		robotRotate = new RobotRotate();
		climber = new Climber();

		if (useVision)
			allCameras = new AllCameras();

		Scheduler.getInstance().run();

		buildTrajectory = new BuildTrajectory();

		driveTrainCanBus.resetEncoders();
		driveTrainCanBus.configOpenLoopAcceleration(.5);
		driveTrainCanBus.configClosedLoopAcceleration(.5);
		driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);

		SmartDashboard.putData(driveTrainCanBus);
		SmartDashboard.putData(cubeHandler);
		SmartDashboard.putData(robotRotate);

		// SmartDashboard.putData(Scheduler.getInstance());
		SmartDashboard.putNumber("Target Feet", positionTarget);
		SmartDashboard.putNumber("Position FPS", positionFPS);

		SmartDashboard.putNumber("Target Angle", angleTarget);
		SmartDashboard.putNumber("Orient Rate", orientRate);
		SmartDashboard.putNumber("Wheel Mode", 0);

		SmartDashboard.putNumber("XPixelTarget", 5);
		SmartDashboard.putBoolean("USBOK", checkUsbFilePath());
		// if (!RobotMap.elevatorSwitch.get() && cubeHandler.holdPositionInches != 0
		// && cubeHandler.getElevatorEncoderPosition() != 0) {
		// cubeHandler.resetElevatorPosition();
		// cubeHandler.holdPositionInches = 0;
		// }
		sensors.initAHRS();
		Timer.delay(3);
		sensors.resetGyro();

		firstAutonomousCommandStarted = false;

		// oi = new OI();
		oi = new OIAlt();
		Timer.delay(.1);

		testTrajectoryChooser = new SendableChooser<Integer>();

		testTrajectoryChooser.addDefault("Test", 0);
		testTrajectoryChooser.addObject("LSW_L", 1);
		testTrajectoryChooser.addObject("LSW_L1 REV", 11);
		testTrajectoryChooser.addObject("LSW_L2", 12);

		testTrajectoryChooser.addObject("LSW_C", 2);
		testTrajectoryChooser.addObject("LSW_C1 REV", 21);

		testTrajectoryChooser.addObject("LSW_R", 3);

		testTrajectoryChooser.addObject("RSW_R", 4);
		testTrajectoryChooser.addObject("RSW_R1 REV", 41);
		testTrajectoryChooser.addObject("RSW_R2", 42);

		testTrajectoryChooser.addObject("RSW_C", 5);
		testTrajectoryChooser.addObject("RSW_C1 REV", 51);

		testTrajectoryChooser.addObject("RSW_L", 6);

		testTrajectoryChooser.addObject("LSC_L", 7);
		testTrajectoryChooser.addObject("LSC_R", 8);

		testTrajectoryChooser.addObject("RSC_R", 9);
		testTrajectoryChooser.addObject("RSC_L", 10);

		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		startPositionChooser = new SendableChooser<Integer>();

		startPositionChooser.addDefault("Left", 1);
		startPositionChooser.addObject("Center", 2);
		startPositionChooser.addObject("Right", 3);

		SmartDashboard.putData("Start Position Chooser", startPositionChooser);

		sameSideChooser = new SendableChooser<Integer>();
		sameSideChooser.addDefault("Do Scale if Available", 1);
		sameSideChooser.addObject("Do Switch if Available", 2);
		sameSideChooser.addObject("Cross Line", 3);

		SmartDashboard.putData("Same Side Priority Chooser", sameSideChooser);

		oppositeSidePriorityChooser = new SendableChooser<Integer>();

		oppositeSidePriorityChooser.addDefault("Do Opposite Scale", 1);
		oppositeSidePriorityChooser.addObject("Do Opposite Switch", 2);
		oppositeSidePriorityChooser.addObject("Do Not Use Opposite", 3);

		SmartDashboard.putData("Opposite Side Chooser", oppositeSidePriorityChooser);

		timeDelayChooser = new SendableChooser<Double>();

		timeDelayChooser.addDefault("No Delay", 0.);
		timeDelayChooser.addObject("One Second", 1.);
		timeDelayChooser.addObject("Two Seconds", 2.);
		timeDelayChooser.addObject("Three Seconds", 3.);
		timeDelayChooser.addObject("Four Seconds", 4.);
		timeDelayChooser.addObject("Five Seconds", 5.);

		SmartDashboard.putData("Delay Chooser", timeDelayChooser);

	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		trajectoryRunning = false;
		motionCommandComplete = false;
		driveTrainCanBus.leftDriveOut(0);
		driveTrainCanBus.rightDriveOut(0);
		driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		cubeHandler.intakeWheelsTurn(0);
	}

	@Override
	public void disabledPeriodic() {
		updateStatus();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		createElevatorRunFile = false;
		firstAutonomousCommandStarted = false;
		cubeHandler.closeIntakeArms();
		driveTrainCanBus.setLeftBrakeMode(true);
		driveTrainCanBus.setRightBrakeMode(true);
		driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		startPosition = startPositionChooser.getSelected();

		switch (startPosition) {

		case 1:
			leftStartPosition = true;
			centerStartPosition = false;
			rightStartPosition = false;
			break;
		case 2:
			leftStartPosition = false;
			centerStartPosition = true;
			rightStartPosition = false;
			break;
		case 3:
			leftStartPosition = false;
			centerStartPosition = false;
			rightStartPosition = true;
			break;

		default:
			break;

		}

		sameSidePriority = sameSideChooser.getSelected();

		switch (sameSidePriority) {

		case 1:
			sameSideScaleHasPriority = true;
			sameSideSwitchHasPriority = false;
			crossLine = false;
			break;
		case 2:
			sameSideScaleHasPriority = false;
			sameSideSwitchHasPriority = true;
			crossLine = false;
			break;
		case 3:
			sameSideScaleHasPriority = false;
			sameSideSwitchHasPriority = false;
			crossLine = true;
			break;

		default:
			sameSideScaleHasPriority = false;
			sameSideSwitchHasPriority = false;
			crossLine = true;
			break;
		}

		oppositeSidePriority = oppositeSidePriorityChooser.getSelected();

		switch (oppositeSidePriority) {

		case 1:
			oppositeScaleHasPriority = false;
			oppositeSwitchHasPriority = true;
			oppositeDoNotUse = false;
			break;
		case 2:
			oppositeScaleHasPriority = false;
			oppositeSwitchHasPriority = false;
			oppositeDoNotUse = true;
			break;

		default:
			oppositeScaleHasPriority = false;
			oppositeSwitchHasPriority = false;
			oppositeDoNotUse = true;
			break;
		}

		timeDelaySeconds = timeDelayChooser.getSelected();

		DriverStation.Alliance color;
		color = DriverStation.getInstance().getAlliance();
		if (color == DriverStation.Alliance.Blue) {
			onBlueAlliance = true;
			onRedAlliance = false;
		}
		if (color == DriverStation.Alliance.Red) {
			onRedAlliance = true;
			onBlueAlliance = false;
		}
		// loop until game data is available
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putString("Auto/gameData", gameData);

		if (gameData.charAt(0) == 'L') {
			// Put left auto code here
			leftSwitchActive = true;
			rightSwitchActive = false;
		} else {
			// Put right auto code here
			leftSwitchActive = false;
			rightSwitchActive = true;
		}
		if (gameData.charAt(1) == 'L') {
			// Put left auto code here
			leftScaleActive = true;
			rightScaleActive = false;

		} else {
			// Put right auto code here
			leftScaleActive = false;
			rightScaleActive = true;

		}
		if (gameData.charAt(2) == 'L') {
			// Put left auto code here
			otherLeftSwitchActive = true;
			otherRightSwitchActive = false;
		} else {
			// Put right auto code here
			otherLeftSwitchActive = false;
			otherRightSwitchActive = true;
		}

		sameSideSwitchAvailable = (leftStartPosition && leftSwitchActive) || (rightStartPosition && rightSwitchActive);
		sameSideScaleAvailable = (leftStartPosition && leftScaleActive) || (rightStartPosition && rightScaleActive);
		sameSideAvailable = sameSideSwitchAvailable || sameSideScaleAvailable;

		if (sameSideSwitchAvailable) {

			doLeftSwitchFromLeft = leftStartPosition && leftSwitchActive
					&& (sameSideSwitchHasPriority || !leftScaleActive);

			doRightSwitchFromRight = rightStartPosition && rightSwitchActive
					&& (sameSideSwitchHasPriority || !rightScaleActive);
		}

		if (sameSideScaleAvailable) {

			doLeftScaleFromLeft = leftStartPosition && leftScaleActive
					&& (sameSideScaleHasPriority || !leftSwitchActive);

			doRightScaleFromRight = rightStartPosition && rightScaleActive
					&& (sameSideScaleHasPriority || !rightSwitchActive);

		}

		if (!sameSideAvailable && !oppositeDoNotUse) {

			doLeftSwitchFromRight = rightStartPosition && oppositeSwitchHasPriority;

			doRightSwitchFromLeft = leftStartPosition && oppositeSwitchHasPriority;

			doLeftScaleFromRight = rightStartPosition && oppositeScaleHasPriority;

			doRightScaleFromLeft = leftStartPosition && oppositeScaleHasPriority;
		}

		doLeftSwitchFromCenter = centerStartPosition && leftSwitchActive;

		doRightSwitchFromCenter = centerStartPosition && rightSwitchActive;

		doCrossLine = !centerStartPosition && (crossLine || (!sameSideAvailable && oppositeDoNotUse));

		isSwitch = false;
		isScale = false;

		isSwitch = doLeftSwitchFromLeft || doLeftSwitchFromCenter || doLeftSwitchFromRight || doRightSwitchFromRight
				|| doRightSwitchFromCenter || doRightSwitchFromLeft;
		isScale = doLeftScaleFromLeft || doLeftScaleFromRight | doRightScaleFromRight || doRightScaleFromLeft;

		startTime = Timer.getFPGATimestamp();

		doMotionOption = true;// will be turned to false if files found

		if (doLeftSwitchFromLeft) {
			PathSelectAuto.LEFTSWITCHFROMLEFT.build();
			if (oppositeDoNotUse)
				secondAutonomousCommand = new DoLeftSwitchFromLeft();
			doLeftSwitchFromLeft = false;
		}

		if (doLeftSwitchFromCenter) {
			PathSelectAuto.LEFTSWITCHFROMCENTER.build();
			doLeftSwitchFromCenter = false;
		}

		if (doLeftSwitchFromRight) {
			PathSelectAuto.LEFTSWITCHFROMRIGHT.build();
			doLeftSwitchFromRight = false;
		}

		if (doRightSwitchFromRight) {
			PathSelectAuto.RIGHTSWITCHFROMRIGHT.build();
			if (oppositeDoNotUse)
				secondAutonomousCommand = new DoRightSwitchFromRight();
			doRightSwitchFromRight = false;
		}

		if (doRightSwitchFromCenter) {
			PathSelectAuto.RIGHTSWITCHFROMCENTER.build();
			doRightSwitchFromCenter = false;
		}

		if (doRightSwitchFromLeft) {
			PathSelectAuto.RIGHTSWITCHFROMLEFT.build();
			doRightSwitchFromLeft = false;
		}

		if (doLeftScaleFromLeft) {
			PathSelectAuto.LEFTSCALEFROMLEFT.build();
			doLeftScaleFromLeft = false;
		}

		if (doLeftScaleFromRight) {
			PathSelectAuto.LEFTSCALEFROMRIGHT.build();
			doLeftScaleFromRight = false;
		}

		if (doRightScaleFromRight) {
			PathSelectAuto.RIGHTSCALEFROMRIGHT.build();
			doRightScaleFromRight = false;
		}

		if (doRightScaleFromLeft) {
			PathSelectAuto.RIGHTSCALEFROMLEFT.build();
			doRightScaleFromLeft = false;
		}

		// ***********************************************************************
		// Cross Line
		if (doCrossLine) {
			firstAutonomousCommand = new DoCrossLineMove();
			secondAutonomousCommand = new TimeDelay(1);
			doCrossLine = false;
			crossLine = false;
			oppositeDoNotUse = false;
		}

		motionCommandComplete = false;
		firstAutonomousCommandStarted = false;
		firstAutonomousCommandDone = false;
		motionCommandRunning = false;
		trajectoryRunning = false;
		autonomousSequenceStarted = false;
		secondAutonomousCommandStarted = false;
		secondAutonomousCommandsDone = false;

		if (useVision)
			allCameras.cubeVisionTurnedOn = true;

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		/*
		 * Autonomous starts off with waiting for any time delay chose. At that point
		 * either a Pathfinder trajectory or a sequence of motios will run. Either one
		 * will bring the robot to the same position and orientation. The cube is then
		 * ejected. These are considered the first autonomous command. When finished, if
		 * it is a scale sequence, a separate command will raise the elevator and sense
		 * the scale distance.
		 * 
		 * Control now passes to the second autonomous command which is a set of events
		 * specific to the destination.
		 * 
		 * These include retracting, turning then driving to vision, picking up a second
		 * cube and delivering it to the same destination.
		 * 
		 */

		Scheduler.getInstance().run();

		if (!autonomousSequenceStarted && Timer.getFPGATimestamp() - startTime > timeDelaySeconds) {

			if (firstAutonomousCommand != null && !firstAutonomousCommandStarted) {
				firstAutonomousCommand.start();
				firstAutonomousCommandStarted = true;
				autonomousSequenceStarted = true;
			}
		}

		// moves to switch or scale are complete so do remainder of auto
		if ((secondAutonomousCommand != null && !secondAutonomousCommandStarted) && firstAutonomousCommandDone) {
			secondAutonomousCommand.start();
			secondAutonomousCommandStarted = true;
		}
		// if (secondAutonomousCommand.isRunning() && allCameras.visionTargetNotFound)
		// secondAutonomousCommand.cancel();

		updateStatus();
	}

	@Override
	public void teleopInit() {

		if (firstAutonomousCommand != null)
			firstAutonomousCommand.cancel();
		if (secondAutonomousCommand != null)
			secondAutonomousCommand.cancel();

		if (useVision)
			allCameras.cubeVisionTurnedOn = false;

		createElevatorRunFile = true;
		trajectoryRunning = false;
		motionCommandComplete = false;
		firstAutonomousCommandDone = false;
		secondAutonomousCommandsDone = false;
		motionCommandRunning = false;
		driveTrainCanBus.configDrivePeakout(DriveTrainCanBus.MAX_ROBOT_FT_PER_SEC, driveSide.both);
		driveTrainCanBus.leftDriveOut(0);
		driveTrainCanBus.rightDriveOut(0);
		cubeHandler.holdPositionInches = cubeHandler.getElevatorPositionInches();
		Robot.cubeHandler.closeIntakeArms();

		// new RunFromGamepadCanBus().start();
		isSwitch = false;
		isScale = false;

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		// System.out.println("==============+++==============");
		// System.out.print("RIO CPU Load: ");
		// System.out.println(loadMon.getCPULoadPct());
		// System.out.print("RIO MEM Load: ");
		// System.out.println(loadMon.getMemLoadPct());
		// System.out.println("===============================\n\n\n");
		Scheduler.getInstance().run();

		if (doTeleopPosition) {
			positionTarget = SmartDashboard.getNumber("Target Feet", 5);
			positionFPS = SmartDashboard.getNumber("Position FPS", 12);
			new DriveToPosition(positionTarget, motionType.absolute, positionFPS, true, false, 8).start();
			doTeleopPosition = false;
		}

		if (doTeleopMagicMotion) {
			positionTarget = SmartDashboard.getNumber("Target Feet", 5);
			positionFPS = SmartDashboard.getNumber("Position FPS", 12);
			new DriveMagicMotion(positionTarget, motionType.absolute, driveSide.both, positionFPS, 8).start();
			doTeleopMagicMotion = false;
		}
		if (doTeleopVisionMotion) {
			positionTarget = SmartDashboard.getNumber("Target Feet", 5);
			positionFPS = SmartDashboard.getNumber("Position FPS", 12);
			visionTarget = (int) SmartDashboard.getNumber("XPixelTarget", 0);
			new DriveToVisionTarget(visionTarget, positionFPS, 8).start();
			doTeleopVisionMotion = false;
		}
		if (doTeleopOrient) {
			// sensors.resetGyro();
			driveTrainCanBus.resetEncoders();
			angleTarget = SmartDashboard.getNumber("Target Angle", 90);
			orientRate = SmartDashboard.getNumber("Orient Rate", .25);
			new RobotOrient(angleTarget, orientRate, true, 5).start();
			doTeleopOrient = false;
		}
		if (doTeleopReverseOrient) {
			sensors.resetGyro();
			driveTrainCanBus.resetEncoders();
			angleTarget = SmartDashboard.getNumber("Target Angle", 90);
			orientRate = SmartDashboard.getNumber("Orient Rate", .25);
			new RobotOrient(-angleTarget, orientRate, true, 5).start();
			doTeleopReverseOrient = false;
		}
		if ((doTeleopTrajectory || doTeleopRevTrajectory) && !trajectoryRunning) {
			leftStartPosition = false;
			centerStartPosition = false;
			rightStartPosition = false;
			isSwitch = false;
			isScale = false;
			secondaryTrajectory = false;
			oppositeSideSwitch = false;

			testTrajectory = testTrajectoryChooser.getSelected();

			switch (testTrajectory) {

			case 0:
				trajFileName = "Test";
				break;
			case 1:
				trajFileName = "LSW_L";
				leftStartPosition = true;
				isSwitch = true;
				oppositeSideSwitch = false;
				break;
			case 11:
				trajFileName = "LSW_L1";
				leftStartPosition = true;
				secondaryTrajectory = true;
				isSwitch = true;
				break;
			case 12:
				trajFileName = "LSW_L2";
				leftStartPosition = true;
				secondaryTrajectory = true;
				isSwitch = true;
				break;

			case 2:
				trajFileName = "LSW_C";
				centerStartPosition = true;
				isSwitch = true;
				oppositeSideSwitch = false;
				break;
			case 21:
				trajFileName = "LSW_C1";
				centerStartPosition = true;
				isSwitch = true;
				break;

			case 3:
				trajFileName = "LSW_R";
				oppositeSideSwitch = true;
				rightStartPosition = true;
				isSwitch = true;
				oppositeSideSwitch = true;
				break;
			case 4:
				trajFileName = "RSW_R";
				rightStartPosition = true;
				isSwitch = true;
				oppositeSideSwitch = false;
				break;
			case 41:
				trajFileName = "RSW_R1";
				rightStartPosition = true;
				isSwitch = true;
				secondaryTrajectory = true;
				break;
			case 42:
				trajFileName = "RSW_R2";
				isSwitch = true;
				rightStartPosition = true;
				secondaryTrajectory = true;
				break;

			case 5:
				trajFileName = "RSW_C";
				centerStartPosition = true;
				isSwitch = true;
				oppositeSideSwitch = false;
				break;
			case 51:
				trajFileName = "RSW_C1";
				centerStartPosition = true;
				isSwitch = true;
				break;

			case 6:
				trajFileName = "RSW_L";
				leftStartPosition = true;
				isSwitch = true;
				oppositeSideSwitch = true;
				break;
			case 7:
				trajFileName = "LSC_L";
				leftStartPosition = true;
				isScale = true;
				break;
			case 8:
				trajFileName = "LSC_R";
				rightStartPosition = true;
				isScale = true;
				break;
			case 9:
				trajFileName = "RSC_R";
				rightStartPosition = true;
				isScale = true;
				break;
			case 10:
				trajFileName = "RSC_L";
				leftStartPosition = true;
				isScale = true;
				break;

			default:
				trajFileName = "Test";
				break;
			}
			if (doTeleopTrajectory) {
				doMotionOption = !checkUsbFilePath();
				if (buildTrajectory.buildFileName(trajFileName, DriveTrainCanBus.Test)) {
					if (!Robot.doMotionOption) {
						constantsFromPrefs();
						driveTrainCanBus.resetEncoders();
						sensors.resetGyro();
						xPosition = Robot.activeLeftTrajectory.get(0).x;
						yPosition = Robot.activeLeftTrajectory.get(0).y - (Robot.driveTrainCanBus.WHEELBASE_WIDTH / 2);

						if (isScale || secondaryTrajectory || isSwitch && oppositeSideSwitch)
							new PathfinderTrajectoryUsingNotifier().start();
						else
							new DoTeleopTrajectorySwitch().start();
						trajectoryRunning = true;
					}
					doTeleopTrajectory = false;
				}
			}
		}

		if (doTeleopRevTrajectory && !trajectoryRunning) {
			doMotionOption = !checkUsbFilePath();
			if (buildTrajectory.buildFileName(trajFileName, DriveTrainCanBus.Test)) {
				if (!Robot.doMotionOption) {
					revConstantsFromPrefs();
					driveTrainCanBus.resetEncoders();
					sensors.resetGyro();
					new PathfinderReverseTrajectoryUsingNotifier().start();
					trajectoryRunning = true;
				}
				doTeleopRevTrajectory = false;
			}
		}
		if (doMotionOption) {
			doTeleopRevTrajectory = false;
			doTeleopTrajectory = false;
		}
		teleopAutoRunning = magicMotionRunning || positionRunning || orientRunning || trajectoryRunning;

		updateStatus();
	}

	/*
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}

	public void updateStatus() {
		SD.putN0("Match Time", DriverStation.getInstance().getMatchTime());
		// SmartDashboard.putData(Scheduler.getInstance());
		updateStatusCounter++;
		if (updateStatusCounter > 6)
			updateStatusCounter = 0;

		switch (updateStatusCounter) {
		case 0:
			// loadMon.updateStatus();

			xPosition += driveMonitor.updateXPosition();
			yPosition += driveMonitor.updateYPosition();

			if (Robot.driveTrainCanBus.runStalledDetect) {
				{
					driveTrainCanBus.leftSideStopped = driveMonitor.getLeftDriveStopped();
					driveTrainCanBus.rightSideStopped = driveMonitor.getRightDriveStopped();
				}
			}
			break;
		case 1:
			driveTrainCanBus.updateStatus();
			// if (Runtime.getRuntime().freeMemory() < minFreeMemory)
			// minFreeMemory = Runtime.getRuntime().freeMemory();
			// if (Runtime.getRuntime().freeMemory() > maxFreeMemory)
			// maxFreeMemory = Runtime.getRuntime().freeMemory();
			// SmartDashboard.putNumber("MinFree", minFreeMemory);
			// SmartDashboard.putNumber("MaxFree", maxFreeMemory);
			// SmartDashboard.putNumber("ActFree", Runtime.getRuntime().freeMemory());
			// SmartDashboard.putNumber("MinMs", minMs);
			// SmartDashboard.putNumber("MaxMs", maxMs);

			break;

		case 2:
			sensors.updateStatus();
			break;
		case 3:
			robotRotate.updateStatus();

			break;

		case 4:
			if (useVision)
				allCameras.updateStatus();
			break;

		case 5:
			SmartDashboard.putBoolean("Loop Closed", closeDriveSpeedLoop);
			SmartDashboard.putBoolean("Log Trajectory", createTrajectoryRunFile);
			SmartDashboard.putBoolean("PositionRunning", positionRunning);
			SmartDashboard.putBoolean("TeleopAutoRunning", teleopAutoRunning);
			SmartDashboard.putBoolean("MagicMotionRunning", magicMotionRunning);
			SmartDashboard.putBoolean("OrientRunning", orientRunning);
			SmartDashboard.putBoolean("VisionRunning", visionMotionRunning);
			SmartDashboard.putString("Chosen Path", chosenFile);
			SmartDashboard.putBoolean("USB?", buildTrajectory.useUSBTrajectories);
			SmartDashboard.putBoolean("Traj Rnng", trajectoryRunning);
			SmartDashboard.putBoolean("Motion Option", doMotionOption);
			SmartDashboard.putBoolean("MotCmdRng", motionCommandRunning);
			SmartDashboard.putBoolean("MotCmdCmplt", motionCommandComplete);
			SmartDashboard.putBoolean("Auto1 Startd", firstAutonomousCommandStarted);
			SmartDashboard.putBoolean("Auto1Dn", firstAutonomousCommandDone);
			SmartDashboard.putBoolean("Auto2Started", secondAutonomousCommandStarted);
			SmartDashboard.putBoolean("Auto2Dn", secondAutonomousCommandsDone);
			SmartDashboard.putNumber("AngTar", angleTarget);
			SmartDashboard.putBoolean("GamepadABXY",
					oi.elevatorToBottomPosition.get() || oi.elevatorToTravelPosition.get()
							|| oi.elevatorToScalePosition.get() || oi.elevatorToScaleLowPosition.get());
			SmartDashboard.putBoolean("GamepadShTr",
					oi.elevatorToSwitchPosition.get() || oi.elevatorToPortalPosition.get()
							|| oi.elevatorToExchangePosition.get() || oi.raiseClimbHook.get());
			SmartDashboard.putNumber("X Position", xPosition);
			SmartDashboard.putNumber("Y Position", yPosition);
			SmartDashboard.putNumber("Active PathP", activeTrajectoryGains[0]);
			SmartDashboard.putNumber("Active PathTurn", activeTrajectoryGains[3]);
			break;
		case 6:
			cubeHandler.updateStatus();
			break;
		default:
			updateStatusCounter = 0;
			break;
		}

		joystickSlider = oi.joystick1.getRawAxis(3);

	}

	public void makePrefs(Preferences prefs, String[] names, double[] values) {
		if (names.length == values.length) {
			for (int i = 0; i < names.length; i++) {
				if (!prefs.containsKey(names[i]))
					prefs.putDouble(names[i], values[i]);
			}
		} else
			DriverStation.reportError("Prefs Array Mismatch" + names, false);
	}

	private void constantsFromPrefs() {
		activeTrajectoryGains[0] = prefs.getDouble("PathP", DriveTrainCanBus.drivePrefsDefaults[14]);
		activeTrajectoryGains[1] = prefs.getDouble("PathD", DriveTrainCanBus.drivePrefsDefaults[15]);
		activeTrajectoryGains[2] = prefs.getDouble("PathA", DriveTrainCanBus.drivePrefsDefaults[16]);
		activeTrajectoryGains[3] = prefs.getDouble("PathTurn", DriveTrainCanBus.drivePrefsDefaults[17]);
	}

	private void revConstantsFromPrefs() {
		activeTrajectoryGains[0] = prefs.getDouble("RevPathP", DriveTrainCanBus.drivePrefsDefaults[18]);
		activeTrajectoryGains[1] = prefs.getDouble("RevPathD", DriveTrainCanBus.drivePrefsDefaults[19]);
		activeTrajectoryGains[2] = prefs.getDouble("RevPathA", DriveTrainCanBus.drivePrefsDefaults[20]);
		activeTrajectoryGains[3] = prefs.getDouble("RevPathTurn", DriveTrainCanBus.drivePrefsDefaults[21]);
	}

	public static boolean checkUsbFilePath() {
		File usbFile = new File("/U/TrajCSV/");
		return usbFile.exists();
	}

}
