package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class VariableOuttake extends InstantCommand {
	private double variable;
    public VariableOuttake() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        
    }

    // Called once when the command executes
    protected void initialize() {
    	System.out.print(variable);
    	variable = -Robot.oi.joystick1.getRawAxis(3);
    	if(variable >= .50) {
    		Robot.cubeHandler.intakeWheelsTurn(CubeHandler.CUBE_SLOW_OUTTAKE);
    	}else {
    		Robot.cubeHandler.intakeWheelsTurn(CubeHandler.CUBE_FAST_OUTTAKE);
    	}
    	
    }

}
