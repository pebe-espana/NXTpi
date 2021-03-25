import lejos.nxt.*;
import lejos.nxt.addon.CompassMindSensor;


/**
 * @author Peter;
 * class to capture the rover hardware configuration through a build reference
 * idea is to be able to have 
 * 	- a one time declaration in package
 * 	- allow one place alteration in case of hardware adaptations
 * this class is collecting static variables across the project
 *
 */
public class NXTplatform {

	 // declared static to be once for all instances of MyNXT
	 
	static String build = "NXT_pi_ROS"; // a descriptive model number for this robot build for version control
	 
	// ---- Allocate sensor ports ----
	static CompassMindSensor cmps = new CompassMindSensor(SensorPort.S1,0x02);
	static UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S4); 
	
	// --- declare state variables --- 
	static boolean exitButton = false;
	static boolean leftButton = false;
	static boolean rightButton = false;
	static boolean forwardButton = false;

}
