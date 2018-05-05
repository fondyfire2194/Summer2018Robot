package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class LowerWings extends InstantCommand {

    public LowerWings() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	RobotMap.wingShifter.set(DoubleSolenoid.Value.kReverse);
    }

}
