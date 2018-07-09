package org.usfirst.frc.team2194.robot;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

public class BuildTrajectory {
	public static File myLeftFile;
	public static File myRightFile;
	public boolean useUSBTrajectories = true;

	public BuildTrajectory() {

	}

	public boolean buildFileName(String name, double[] gains) {

		Robot.doMotionOption = true;

		String tempPath = null;
		String filePath = "/home/lvuser/TrajCSV/";
		String testPath = "Test/";
		String switchPath = "Switch/";
		String scalePath = "Scale/";

		if (useUSBTrajectories)
			tempPath = Robot.usbFilePath;
		else
			tempPath = filePath;

		if (name == "Test")
			tempPath += testPath;
		else {
			if (Robot.leftStartPosition)
				tempPath += "LeftStart/";
			if (Robot.centerStartPosition)
				tempPath += "CenterStart/";
			if (Robot.rightStartPosition)
				tempPath += "RightStart/";
		}

		if (Robot.isSwitch)
			tempPath += switchPath;

		if (Robot.isScale)
			tempPath += scalePath;

		myLeftFile = new File(tempPath + name + "_left_detailed.csv");
		myRightFile = new File(tempPath + name + "_right_detailed.csv");

		if (myLeftFile.exists() && myRightFile.exists()) {
			Robot.activeLeftTrajectory = Pathfinder.readFromCSV(myLeftFile);
			Robot.activeRightTrajectory = Pathfinder.readFromCSV(myRightFile);
			Robot.doMotionOption = false;
			Robot.chosenFile = name;
		}

		DriverStation.reportWarning(myLeftFile.getAbsolutePath(), false);
		DriverStation.reportWarning(myLeftFile.toString(), false);

		if (!Robot.doMotionOption) {
			for (int i = 0; i < Robot.activeTrajectoryGains.length; i++) {
				Robot.activeTrajectoryGains[i] = gains[i];
			}
		}

		return !Robot.doMotionOption;
	}
}
