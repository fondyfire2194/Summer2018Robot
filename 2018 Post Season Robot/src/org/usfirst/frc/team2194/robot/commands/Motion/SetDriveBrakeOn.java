package org.usfirst.frc.team2194.robot.commands.Motion;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetDriveBrakeOn extends InstantCommand {
private boolean myBrakeOn;
    public SetDriveBrakeOn(boolean on) {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        myBrakeOn=on;
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.driveTrainCanBus.setBrakeMode(myBrakeOn);
    }

}
