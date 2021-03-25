/* not needed on NXT version
 * package NXT_definitions;
 */

public interface CommandConstants {

		// update Jan 2021
	
		// needed in various parts
		// on NXT side: Robot, MessageStructure
		// on Mac/PC side: Main prg, MessageStructure
	
		public static final int MESSAGE_LENGTH = 41;
		public static final int COMMAND_LENGTH = 4;
		public static final int B_COMMAND_LENGTH = 1 + 3*5; // number of bytes needed to be transferred
															// cmd = 1 byte (255 possibilities) params = 5 bytes for 5 decimals
															
		public static final int B_MESSAGE_LENGTH = 41*5; 	// maximum number of bytes needed to be transferred
															// CMD_xx_N below gives command specific integer values expected back
															// each integer with 5 bytes for 5 decimal numbers 0-9, including -sign 
		
		// commands and answers expected
		public static final int COMMAND_BATTERY_VOLTAGE = 0;
		public static final int CMD_BV_N = 1; // one integer answer expected
		// send back up to 8 distance values after a ping of ultrasound, plus compass reading, plus one spare extra still unused 
		public static final int COMMAND_PING = 1;
		public static final int CMD_P_N = 1; // 8 + 1;
		// do a differential pilot travel, (param 1 = distance, param 2 = speed) followed by ping 
		public static final int COMMAND_TRAVEL = 2;
		public static final int CMD_TR_N =  8 + 1;
		// do a differential pilot turn, (param 1 = angle, param 2 = rotate speed) followed by ping 
		public static final int COMMAND_TURN = 3;
		public static final int CMD_TN_N = 8 + 1;
		// disconnect bluetooth and stop running
		public static final int COMMAND_DISCONNECT = 4;
		public static final int CMD_D_N = 1;
		// do an arc with radius and final angle specified (see differential pilot description)
		public static final int COMMAND_ARC = 5;
		public static final int CMD_A_N = 8 + 1;
		
		/*
		 *  get collected odometry data (provides us_sound in first five message parameters, 
		 *  and odometry in next three (x,y,head * 10), then compass (*10)
		 */
		public static final int COMMAND_ODO = 6;
		public static final int CMD_O_N = 5 + 3 + 1;
		
		/*
		 * still undefined - want to get a monte-carlo localisation back ???
		 */
		public static final int COMMAND_LOC = 7;
		public static final int CMD_L_N = 2;
		
		/*
		 *  get compass behaviour: if jointly called with 
		 *  	first parameter = 999, then do calibration run before the default behaviour
		 *  	default behaviour = do 10 turns of angle = param1, and send back 10 compass readings
		 *      second parameter = 999, then return to start position after default 10 turns
		 */
		public static final int COMMAND_CMPS = 8;
		public static final int CMD_CMP_N = 1; //10;
		
		/*
		 * NXT Cams analysis of - downfacing camera up to 8 rectangles identified
		 * no of rectangles, (colour, x,y,w,h) = 1 + 5x8
		 */
		public static final int COMMAND_CAMS = 9;
		public static final int CMD_CAMS_N = 41;
		
		// length of answer expected for the commands given
		public static final int[] CMD_ANS_N = { CMD_BV_N, CMD_P_N, CMD_TR_N, CMD_TN_N, CMD_D_N, CMD_A_N, CMD_O_N, CMD_L_N, CMD_CMP_N, CMD_CAMS_N };
		
}



