package org.usfirst.frc.team2194.robot.commands.CubeHandler;

import org.usfirst.frc.team2194.robot.commands.AlertDriver;
import org.usfirst.frc.team2194.robot.commands.TimeDelay;
import org.usfirst.frc.team2194.robot.subsystems.CubeHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class IntakeCube extends CommandGroup {

    public IntakeCube() {
    	    	
    	addSequential(new TurnIntakeWheelsOneShot());
    	addSequential(new AlertDriver("Intake"));
    	addSequential(new CloseIntakeArms());
    	addSequential(new AlertDriver("Close"));
    	addSequential(new TimeDelay(1.0));
    	addParallel(new TurnIntakeWheels(0));
    	addSequential(new AlertDriver("Off"));
    
   
   
    	
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
