package org.usfirst.frc.team2194.robot.commands.Autonomous;

import org.usfirst.frc.team2194.robot.DistCon;
import org.usfirst.frc.team2194.robot.Robot.motionType;
import org.usfirst.frc.team2194.robot.commands.CubeHandler.ElevatorMoveToHeight;
import org.usfirst.frc.team2194.robot.commands.Motion.DriveToPosition;
import org.usfirst.frc.team2194.robot.commands.Motion.RobotOrient;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;
import org.usfirst.frc.team2194.robot.subsystems.DriveTrainCanBus.driveSide;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoLeftSwitchFromRight extends CommandGroup {

	public DoLeftSwitchFromRight() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// retract from switch and lower elevator

		addSequential(new DriveToPosition(0, motionType.absolute, DistCon.SHORT_POSITION_RATE, false, 3));
		addSequential(new RobotOrient(-90, DistCon.ORIENT_RATE, driveSide.both, false, 2));
		addParallel(new ElevatorMoveToHeight(CubeHandler.ELEVATOR_PICKUP_POSITION_INCHES));

	}
}
