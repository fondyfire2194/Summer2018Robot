package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class EndCommand extends InstantCommand {

    public EndCommand() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrainCanBus);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.endCommand = true;
    	Timer.delay(1);
    	Robot.endCommand = false;
    }

}
