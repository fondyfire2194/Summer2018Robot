package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.*;
import org.usfirst.frc.team2194.robot.commands.Autonomous.*;

public enum PathSelectAuto {
	/*
	 * Order of parameters is Trajectory File, trajectory Gains, continuing side and
	 * angle for switch only, first move command,second trajectory command, second
	 * move command
	 * 
	 */
	LEFTSWITCHFROMCENTER("LSW_C", DriveTrainCanBus.LSW_C, driveSide.left, 0, new DoLeftSwitchFromCenterMove(),
			new DoLeftSwitchFromCenterUsingTrajectories(), new DoLeftSwitchFromCenter()), //
	RIGHTSWITCHFROMCENTER("RSW_C", DriveTrainCanBus.RSW_C, driveSide.right, 0, new DoRightSwitchFromCenterMove(),
			new DoRightSwitchFromCenterUsingTrajectories(), new DoRightSwitchFromCenter()), //
	LEFTSWITCHFROMLEFT("LSW_L", DriveTrainCanBus.LSW_L, driveSide.left, 90, new DoLeftSwitchFromLeftMove(),
			new DoLeftSwitchFromLeftUsingTrajectories(), new DoLeftSwitchFromLeft()), //
	RIGHTSWITCHFROMRIGHT("RSW_R", DriveTrainCanBus.RSW_R, driveSide.right, -90, new DoRightSwitchFromRightMove(),
			new DoRightSwitchFromRightUsingTrajectories(), new DoRightSwitchFromRight()), //
	LEFTSWITCHFROMRIGHT("LSW_R", DriveTrainCanBus.LSW_R, driveSide.both, 180, new DoLeftSwitchFromRightMove(),
			new DoLeftSwitchFromRight(), new DoLeftSwitchFromRight()), //
	RIGHTSWITCHFROMLEFT("RSW_L", DriveTrainCanBus.RSW_L, driveSide.both, 180, new DoRightSwitchFromLeftMove(),
			new DoRightSwitchFromLeft(), new DoRightSwitchFromLeft()), //
	/*
	 * scales
	 */
	LEFTSCALEFROMLEFT("LSC_L", DriveTrainCanBus.LSC_L, new DoLeftScaleFromLeftMove(), new DoLeftScale(),
			new DoLeftScale()), //
	RIGHTSCALEFROMRIGHT("RSC_R", DriveTrainCanBus.RSC_R, new DoRightScaleFromRightMove(), new DoRightScale(),
			new DoRightScale()), //
	LEFTSCALEFROMRIGHT("LSC_R", DriveTrainCanBus.LSC_R, new DoLeftScaleFromRightMove(), new DoLeftScale(),
			new DoLeftScale()), //
	RIGHTSCALEFROMLEFT("RSC_L", DriveTrainCanBus.RSC_L, new DoRightScaleFromLeftMove(), new DoRightScale(),
			new DoRightScale());

	private String name = null;
	private double[] gains = { 0, 0, 0, 0 };
	private driveSide side;
	private double continuingAngle;
	private Command firstMove;
	private Command second;
	private Command secondMove;

	/*
	 * scale constructor
	 */
	PathSelectAuto(String name, double[] gains, Command firstMove, Command second, Command secondMove) {
		this.name = name;
		this.gains = gains;
		this.firstMove = firstMove;
		this.second = second;
		this.secondMove = secondMove;
	}

	/*
	 * switch constructor
	 */
	PathSelectAuto(String name, double[] gains, driveSide side, double continuingAngle, Command firstMove,
			Command second, Command secondMove) {
		this.name = name;
		this.gains = gains;
		this.side = side;
		this.continuingAngle = continuingAngle;
		this.firstMove = firstMove;
		this.second = second;
		this.secondMove = secondMove;
	}

	void build() {
		if (Robot.getUSBPresence() && Robot.buildTrajectory.buildFileName(name, gains)) {
			if (Robot.isSwitch) {
				Robot.continuingSide = side;
				Robot.continuingAngle = continuingAngle;
				Robot.firstAutonomousCommand = new DoTrajectorySwitch();
			}
			if (Robot.isScale)
				Robot.firstAutonomousCommand = new DoTrajectoryScale();

			Robot.secondAutonomousCommand = second;
			Robot.trajectoryRunning = true;
		}

		else {
			Robot.firstAutonomousCommand = firstMove;
			Robot.motionCommandRunning = true;
			Robot.secondAutonomousCommand = secondMove;
		}
	}

}
