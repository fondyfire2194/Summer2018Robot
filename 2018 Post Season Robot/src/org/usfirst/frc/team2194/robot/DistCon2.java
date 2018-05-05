package org.usfirst.frc.team2194.robot;

public class DistCon2 {
	/*
	 * ongoing moves for second cube after delivering first cube
	 * 
	 */
	public DistCon2() {

	};

	/*
	 * center scale either side - just change angle sign distances in decimal feet
	 * reverse distances to switch turn to 0 also
	 */
	public static double CSW_RD1 = 5;
	public static double CSW_A1 = 42;//sign set in routine
	public static double CSW_TO_CUBE = 4;

	/*
	 * left and right switches from same side
	 * 
	 * 
	 */
	public static double LRSW_RD1 = 1.5;
	public static double LRSW_A1 = 179;
	public static double LRSW_RD2 = 5;
	public static double LRSW_A2 = 135;
	public static double LRSW_TO_CUBE = 3;
	public static double LRSW_TO_SW = 1;
	public static double LRSW_RD3 = 3;

	/*
	 * left and right scales - if switch is open deliver there else hold on to cube
	 * 
	 * 
	 */
	public static double LRSC_RETRACT = 1.5;
	public static double LRSC_A1 = 156;
	public static double LRSC_TO_CUBE = 9;
	public static double LRSC_TO_SW = 1;
	public static double LRSC_RD3 = 3;

}
