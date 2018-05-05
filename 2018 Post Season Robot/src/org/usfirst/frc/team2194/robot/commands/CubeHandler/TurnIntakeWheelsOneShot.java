package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class TurnIntakeWheelsOneShot extends InstantCommand {

    public TurnIntakeWheelsOneShot() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.cubeHandler.intakeWheelsTurn(CubeHandler.INTAKE_SPEED);
    }

}
