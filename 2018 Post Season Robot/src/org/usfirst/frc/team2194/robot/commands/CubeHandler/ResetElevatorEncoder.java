package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ResetElevatorEncoder extends InstantCommand {

    public ResetElevatorEncoder() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.cubeHandler);
    }

    // Called once when the command executes
    protected void initialize() {
    	
    	Robot.cubeHandler.resetElevatorPosition();
    	Robot.cubeHandler.holdPositionInches = 0;
    }

}
