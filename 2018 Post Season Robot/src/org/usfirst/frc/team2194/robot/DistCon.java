package org.usfirst.frc.team2194.robot;

public class DistCon {
	/*
	 * 
	 * distances in inches to match field drawings convert to feet for program use
	 */
	public DistCon() {

	};

	/*
	 * all dimensions in inches. Convert to feet before using in moves
	 */
	private static double robot_width = 35;// includes bumpers
	private static double robot_length = 40;// includes bumbers
	private static double robot_front_extension = 7;
	private static double cube_width = 1 * 12;

	private static double scale_platform_width = 4 * 12;
	private static double scale_platform_depth = 3 * 12;
	private static double width_across_scale = 15 * 12;

	private static double width_across_switch = 12 * 12;
	private static double switch_platform_width = 3 * 12;
	private static double switch_platform_depth = 4 * 12;

	private static double field_side_wall_to_robot_edge_at_start = 29.69;
	private static double field_width = 323;
	private static double field_side_wall_to_switch_wall = 85.25;

	private static double alliance_wall_to_switch_start = 140;
	private static double alliance_wall_to_switch_end = 196;
	private static double alliance_wall_to_floor_platform_edge = 261.47;
	private static double alliance_wall_to_neutral_zone_start = 288;
	private static double alliance_wall_to_scale_platform_edge = 299.65;

	public static double LONG_POSITION_RATE = 8.5;
	public static double SHORT_POSITION_RATE = 5;
	public static double ORIENT_RATE = .5;
	public static double LSW_C_ORIENT_RATE = .6;

	public static double LR_SW_1;
	public static double LR_SW_2;

	public static double LRSW_RL_1;
	public static double LRSW_RL_2;
	public static double LRSW_RL_2_ADDER;
	public static double LRSW_RL_A = 150;

	public static double LRSW_RL_3 = 36 / 12;// inches

	public static double LRSW_RL_A2 = 130;

	// lsw_RL_4 = 96;

	public static double LR_SC_1;
	public static double LR_SC_2;

	public static double LRSC_RL_1;
	public static double LRSC_RL_2;
	public static double LRSC_RL_3;

	public static double LSWC_1;
	public static double LSW_CA;
	public static double LSWC_2;
	public static double LSWC_3;

	public static double RSWC_1;

	public static double CROSS_LINE;

	/*
	 * up field travel distances for any move are calculated from the alliance wall
	 * and then have the robot length/2 subtracted
	 */

	public static void init() {

		/*
		 * field length distances
		 */
		double alliance_wall_to_mid_switch = (alliance_wall_to_switch_start + alliance_wall_to_switch_end) / 2;

		double alliance_wall_to_center_switch_scale_gap_including_cube = (alliance_wall_to_switch_end + cube_width
				+ alliance_wall_to_floor_platform_edge) / 2;

		double alliance_wall_to_scale_platform_center = alliance_wall_to_scale_platform_edge
				+ (scale_platform_width / 2);

		/*
		 * cross field distances
		 */
		double side_start_robot_center_line_from_side_wall = field_side_wall_to_robot_edge_at_start + robot_width / 2;

		/*
		 * same side switch from left or right move forward then turn +-90 move forward
		 * to switch
		 */
		double make_sure_switch_wall = 12;
		LR_SW_1 = (alliance_wall_to_mid_switch - (robot_length / 2)) / 12;

		LR_SW_2 = (make_sure_switch_wall + field_side_wall_to_switch_wall - side_start_robot_center_line_from_side_wall
				- (robot_length / 2)) / 12;

		/*
		 * opposite side switch forward - turn - forward stop with front of robot in
		 * second from end cube gap then turn at angle into cube gap and move forward
		 * until touching switch
		 */
		LRSW_RL_2_ADDER = 30;// inches

		LRSW_RL_1 = (alliance_wall_to_center_switch_scale_gap_including_cube - (robot_length / 2)) / 12;

		LRSW_RL_2 = (LRSW_RL_2_ADDER + (field_width / 2) - side_start_robot_center_line_from_side_wall) / 12;

		LRSW_RL_A = 150;

		LRSW_RL_3 = 36 / 12;// inches

		LRSW_RL_A2 = 130;

		/*
		 * same side scale move dist1 to scale switch gap then moves the remaining
		 * distance at lower speed to scale platform center - turn +/- 90
		 */

		LR_SC_1 = (alliance_wall_to_center_switch_scale_gap_including_cube - (robot_length / 2)) / 12;
		LR_SC_2 = (alliance_wall_to_scale_platform_center - (robot_length / 2)) / 12;

		/*
		 * opposite side scale
		 */

		LRSC_RL_1 = (alliance_wall_to_center_switch_scale_gap_including_cube - (robot_length / 2)) / 12;

		LRSC_RL_2 = (field_width - (1.5 * side_start_robot_center_line_from_side_wall)) / 12;

		LRSC_RL_3 = (alliance_wall_to_scale_platform_center - alliance_wall_to_center_switch_scale_gap_including_cube)
				/ 12;

		/*
		 * 
		 * center switch
		 * 
		 */

		double initial_center_start_left_switch_move = 12;

		double measured_center_start_left_switch_cross_move = 114;// was 150 need 9 ft / cos 15

		/*
		 * forward_distance_in_cross_move = 114 / sin15 = 30" required forward distance
		 * = 140 - robot_length = 100" 100 - 12 - 30 = 48
		 * 
		 * 
		 */

		double measured_center_start_left_switch_final_move = 52;

		// left switch from center

		LSWC_1 = initial_center_start_left_switch_move / 12;

		LSW_CA = -75;

		LSWC_2 = measured_center_start_left_switch_cross_move / 12;

		LSWC_3 = measured_center_start_left_switch_final_move / 12;

		// right switch from center

		RSWC_1 = .5 + ((alliance_wall_to_switch_start - robot_length) / 12);

		CROSS_LINE = LR_SW_1;
	}

	public static void updateStatus() {

		SD.putN2("SideSwitch1", LR_SW_1);
		SD.putN2("SideSwitch2", LR_SW_2);

		SD.putN2("OppSideSwitch1", LRSW_RL_1);
		SD.putN2("OppSideSwitch2", LRSW_RL_2);
		SD.putN2("OppSideSwitch3", LRSW_RL_3);

		SD.putN2("LeftCenterSwitch1", LSWC_1);
		SD.putN2("LeftCenterSwitch2", LSWC_2);
		SD.putN2("LeftCenterSwitch3", LSWC_3);
		SD.putN2("LeftCenterSwitchAngle", LSW_CA);

		SD.putN2("RightCenterSwitch1", RSWC_1);

		SD.putN2("SideScale1", LR_SC_1);
		SD.putN2("SideScale2", LR_SC_2);

		SD.putN2("OppSideScale1", LRSC_RL_1);
		SD.putN2("OppSideScale2", LRSC_RL_2);
		SD.putN2("OppSideScale3", LRSC_RL_3);

	}
}
