package org.usfirst.frc.team2194.robot;

import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoLeftScaleFromLeftMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoLeftScaleFromRightMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoLeftSwitchFromCenterMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoLeftSwitchFromLeftMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoLeftSwitchFromRightMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoRightScaleFromLeftMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoRightScaleFromRightMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoRightSwitchFromCenterMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoRightSwitchFromLeftMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoRightSwitchFromRightMove;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoTrajectoryScale;
import org.usfirst.frc.team2194.robot.commands.AutoMoves.DoTrajectorySwitch;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoLeftScale;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoLeftSwitchFromCenter;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoLeftSwitchFromLeft;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoLeftSwitchFromRight;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoRightScale;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoRightSwitchFromCenter;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoRightSwitchFromLeft;
import org.usfirst.frc.team2194.robot.commands.Autonomous.DoRightSwitchFromRight;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus;

import edu.wpi.first.wpilibj.command.Command;

public enum PathSelectAuto {
	/*
	 * Order of parameters is Trajectory File, trajectory Gains, stop side and stop
	 * segmnts for switch only, first move command,second trajectory command, second
	 * move command
	 * 
	 */

	LEFTSWITCHFROMCENTER("LSW_C", DriveTrainCanBus.LSW_C, new DoLeftSwitchFromCenterMove(),
			new DoLeftSwitchFromCenter(), new DoLeftSwitchFromCenter()), //
	RIGHTSWITCHFROMCENTER("RSW_C", DriveTrainCanBus.RSW_C, new DoRightSwitchFromCenterMove(),
			new DoRightSwitchFromCenter(), new DoRightSwitchFromCenter()), //

	LEFTSWITCHFROMLEFT("LSW_L", DriveTrainCanBus.LSW_L, new DoLeftSwitchFromLeftMove(), new DoLeftSwitchFromLeft(),
			new DoLeftSwitchFromLeft()), //
	RIGHTSWITCHFROMRIGHT("RSW_R", DriveTrainCanBus.RSW_R, new DoRightSwitchFromRightMove(),
			new DoRightSwitchFromRight(), new DoRightSwitchFromRight()), //

	LEFTSWITCHFROMRIGHT("LSW_R", DriveTrainCanBus.LSW_R, new DoLeftSwitchFromRightMove(), new DoLeftSwitchFromRight(),
			new DoLeftSwitchFromRight()), //
	RIGHTSWITCHFROMLEFT("RSW_L", DriveTrainCanBus.RSW_L, new DoRightSwitchFromLeftMove(), new DoRightSwitchFromLeft(),
			new DoRightSwitchFromLeft()), //
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
	 * use trajectories for lset and right start switch and for left switch center
	 * start
	 * 
	 * 
	 */
	void build() {
		if (Robot.isSwitch && (!Robot.centerStartPosition || Robot.leftSwitchActive) && Robot.checkUsbFilePath()
				&& Robot.buildTrajectory.buildFileName(name, gains)) {

			if (Robot.isSwitch)
				Robot.firstAutonomousCommand = new DoTrajectorySwitch();

			if (Robot.isScale)
				Robot.firstAutonomousCommand = new DoTrajectoryScale();

			Robot.xPosition = Robot.activeLeftTrajectory.get(0).x;
			Robot.yPosition = Robot.activeLeftTrajectory.get(0).y - (Robot.driveTrainCanBus.WHEELBASE_WIDTH / 2);
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
