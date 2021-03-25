import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.Battery;
import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

/**
 * 	based on NXT-pi_RoverV2 bluetooth experience
 *  rebuild a sensor - task interface with ROS environment
 *  14-1-2021
 */

/**
 * @author Peter
 * date of usage and experimenting : 24/1/2021
 * Excel file with results available.
 */
public class NXT_pi_ROS implements ButtonListener, CommandConstants {

	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub

		NXT_pi_ROS listener = new NXT_pi_ROS();
		Button.ESCAPE.addButtonListener(listener);
		
		DataInputStream _dis = null;
		DataOutputStream _dos = null;
		BTConnection _btc = null;
		
		// =======================================================================
		//      command and message handling parameters/variables
		int _Ncmd = COMMAND_LENGTH;  // length of command message
		int _NBcmd = B_COMMAND_LENGTH;  // length in bytes needed

		int _Nrep = MESSAGE_LENGTH; // length of reply message
		int _NBrep = B_MESSAGE_LENGTH;

		TranslateCMD tcmd = new TranslateCMD();  //not used if command[] not parsed
		TranslateREP trep = new TranslateREP();
		
		float _cmps_value = (float)360.0;
		double realval;
		int i8 = 0;
		int k1,k2;
		int outval;

		// receive _pyScmd array from Raspi and convert these to _command vector 
		int[] _command = new int[_Ncmd]; // updated to 4 - command, distance/angle, speed, radius
		String[] _pyScmd = new String[_NBcmd]; // modified command in bytes = 1 byte + 3*5 bytes

		// build the reply vector and convert to _pyIrep to pass to Raspi 
		// length of _pyIrep is dynamic dependent on which command is answered
		int[] _reply = new int[_Nrep];  // updated 8 distance values, compass, plus 1 for NXT_request_flag
		// or to accommodate 41 camera attributes
		int[] _reply8 = new int[8];  // for 8 distance values
		int[] ans_expected = CMD_ANS_N;
		int[] _pyIrep;    // integer array, where each integer is the ascii code of byte to be sent 
		int n_exp;

		
// 		program flow status messages
		String _connected = "Connected";
		String _waiting = "Waiting for Pi...";
		String _waiting2 = "Waiting for blue...";
		String _sending = "Sending...";
		String _closing = "Closing...";

		boolean _keepItRunning = true;
		int _runtime = 0;
		int _waitcycles = 0; // to add up all waits for one sending cycle
		
		// ----- start now the main prog action ---

		LCD.clear();
		LCD.drawString(_waiting,0,0);
		LCD.refresh();
		
		NXTplatform.sonar.reset();
		//sonar.setMode(1);  //MODE_PING = 1
		NXTplatform.sonar.setMode(2);  //MODE_CONTINUOUS = 2
		
		Sound.beepSequenceUp();  //ready to go

		// ====  Slave waits for Master to connect with a command ===
		_btc = Bluetooth.waitForConnection();
		
		LCD.clear();
		LCD.drawString(_connected,0,0); 
		LCD.refresh();	
		Sound.twoBeeps();     //ready to receive data

		// Set up the data input and output streams
		_dis = _btc.openDataInputStream();
		_dos = _btc.openDataOutputStream();
		
		
		_keepItRunning = true;
		
		while (_keepItRunning) {   
			
			_waitcycles = 0; //set cumulative wait zero at start of one loop
			// ===== receive data from pi in order to sync start of data sequencing =================================
			
			/* 
			 * PiServer sends up to 4 encoded two bytes of cmd length plus byte char(48+k)
			 */
			
			LCD.drawString(_waiting,0,1);
			
			//if a four integer command sequence is expected
			/*
			for(int k = 0; k<4; k++){  
				_pyScmd[k] = Byte.toString(_dis.readByte());    // receives bytes 49 -> Strings '49',50,51,52
				//Sound.beep();
			
				LCD.drawString(_pyScmd[k], 0, k+1);
				_command[k] = Integer.parseInt(_pyScmd[k]);     // parsed integers are 49,50,51,52
				LCD.drawInt(_command[k], 6, k+2);
			
				// wait before next byte to be sent from pi
				try{Thread.sleep(100);}
				catch(InterruptedException e){
					System.exit(0);
				}
				_waitcycles = _waitcycles + 100;
			}
			*/
			
			_pyScmd[0] = Byte.toString(_dis.readByte());    // receives bytes 49 -> Strings '49',50,51,52
			LCD.drawString(_pyScmd[0], 0, 2);
			_command[0] = Integer.parseInt(_pyScmd[0]);     // parsed integers are 49,50,51,52
			LCD.drawInt(_command[0], 6, 2);
			// wait a while before continuing
			try{Thread.sleep(100);}
			catch(InterruptedException e){
				System.exit(0);
			}
			_waitcycles = _waitcycles + 100;
			

		// ===== send data - WHILE LOOP for duration of main program ======================

			LCD.drawString(_sending,0,3);
			
			// reset all parts of reply to 0
			for(int n = 0; n < _Nrep; n++){
				_reply[n] = 0;
			}
			
			_command[0] = COMMAND_BATTERY_VOLTAGE;
			_reply[0] = Battery.getVoltageMilliVolt();  //integer in units of mV
			outval = _reply[0];
			_dos.flush();
		
			
			// parse each integer into decimal notation 
			// and send each as an ascii character
			// run answer index over the expected answer size <= _Nrep
			// answer between 1 and 41 decimals (including a - sign as possibility)
			// so always between 5 and 205 bytes to be sent back
			
			n_exp = 5 * ans_expected[_command[0]];
			if (n_exp > _NBrep) {
				n_exp = _NBrep;
			} else if ( n_exp < 5  ) {
				n_exp = 5;
			}
			_pyIrep = new int[n_exp];
			_pyIrep = trep.getREP(_reply, _command[0], n_exp);
			
			for (int k=0 ; k < n_exp ; k++){
				// do a check on data to be sent
				if (_pyIrep[k] > 57) { // 57 = 9 in ascii
					_pyIrep[k] = 65; // A = error detected
				} else if (_pyIrep[k] < 45 ) { // 45 = - in ascii
					_pyIrep[k] = 65; 
				}
				LCD.drawString("bat",0,4);
				LCD.drawInt(outval,0,5);
				_dos.writeByte(_pyIrep[k]);
			}
			
			
			// wait before sending more info back to pi
			try{Thread.sleep(100);}
			catch(InterruptedException e){
				System.exit(0);
			}
			_waitcycles = _waitcycles + 100;
			
			//---------------------------------------------------------------------------
			
			_command[0] = COMMAND_CMPS;
			_cmps_value = NXTplatform.cmps.getDegrees();
			realval = 10.0 *_cmps_value;
			_reply[0] = (int)(realval);  //integer in units of 0.1 degree
			outval = _reply[0];
			_dos.flush();
			
			// parse each integer into decimal notation 
			// and send each as an ascii character
			// run answer index over the expected answer size <= _Nrep
			// answer between 1 and 41 decimals (including a - sign as possibility)
			// so always between 5 and 205 bytes to be sent back
			n_exp = 5 * ans_expected[_command[0]];
			if (n_exp > _NBrep) {
				n_exp = _NBrep;
			} else if ( n_exp < 5  ) {
				n_exp = 5;
			}
			_pyIrep = new int[n_exp];
			_pyIrep = trep.getREP(_reply, _command[0], n_exp);
			
			for (int k=0 ; k < n_exp ; k++){
				// do a check on data to be sent
				if (_pyIrep[k] > 57) { // 57 = 9 in ascii
					_pyIrep[k] = 65; // A = error detected
				} else if (_pyIrep[k] < 45 ) { // 45 = - in ascii
					_pyIrep[k] = 65; 
				}
				LCD.drawString("cmp",6,4);
				LCD.drawInt(outval,6,5);
				_dos.writeByte(_pyIrep[k]);
			}
			
			// wait before sending more info back to pi
			try{Thread.sleep(100);}
			catch(InterruptedException e){
				System.exit(0);
			}
			_waitcycles = _waitcycles + 100;
			
			//---------------------------------------------------------------------------
			
			_command[0] = COMMAND_PING;			
			// --- two methods available - one ping or array of 8
			
			
			NXTplatform.sonar.setMode(2);  //MODE_CONTINUOUS = 2
			//sonar.setMode(1);  //MODE_PING = 1
			//NXTplatform.sonar.ping();
			
			
			try{Thread.sleep(20);}
			catch(InterruptedException e){
				System.exit(0);
			}
			_waitcycles = _waitcycles + 20;
			
			//_reply[0] = NXTplatform.sonar.getDistance(); //single first ping result
			//NXTplatform.sonar.getDistances(_reply8);
			for (int i=0; i<8; i++) {
				_reply8[i] = NXTplatform.sonar.getDistance();
				try{Thread.sleep(15);}
				catch(InterruptedException e){
					System.exit(0);
				}
			}
			_waitcycles = _waitcycles + (15*8);
			//sonar.setMode(1);  //MODE_PING = 1
			NXTplatform.sonar.ping();
			//NXTplatform.sonar.getDistances(_reply8);
			
			// output part of sonar signal to screen
			i8 = i8 + 1;
			if (i8 > 7) { i8 = 0; }
			LCD.drawInt(i8, 11, 6);  //which one is shown
			outval = _reply8[i8];
			LCD.drawInt(outval,11,5); //show it
			
			// check plausibility and pick valid values
			_reply[0] = 0;
			k1 = 0;
			k2 = 0;
			// find average over 8 values, ignoring 255
			for (int n = 0; n<8; n++) {
				if (_reply8[n] != 255 ) {
					_reply[0] = _reply[0] + _reply8[n];
					k1 = k1+1;
			    } else {
			    	k2 = k2+1;  // number of 255 found
			    }
			}
			if (k1>0) {
				_reply[0] = _reply[0] / k1;  // integer division! (remainder: a % b)
			} else  {
				_reply[0] = 255;
			}
			_dos.flush();
			
			// parse each integer into decimal notation 
			// and send each as an ascii character
			// run answer index over the expected answer size <= _Nrep
			// answer between 1 and 41 decimals (including a - sign as possibility)
			// so always between 5 and 205 bytes to be sent back
			n_exp = 5 * ans_expected[_command[0]];
			if (n_exp > _NBrep) {
				n_exp = _NBrep;
			} else if ( n_exp < 5  ) {
				n_exp = 5;
			}
			_pyIrep = new int[n_exp];
			_pyIrep = trep.getREP(_reply, _command[0], n_exp);
			
			for (int k=0 ; k < n_exp ; k++){
				// do a check on data to be sent
				if (_pyIrep[k] > 57) { // 57 = 9 in ascii
					_pyIrep[k] = 65; // A = error detected
				} else if (_pyIrep[k] < 45 ) { // 45 = - in ascii
					_pyIrep[k] = 65; 
				}
				LCD.drawString("son",11,4);
				_dos.writeByte(_pyIrep[k]);
			}
			
			// wait before sending more info back to pi
			try{Thread.sleep(100);}
			catch(InterruptedException e){
				System.exit(0);
			}
			_waitcycles = _waitcycles + 100;
			
			//------------------------------------------------------------------------
			
			_dos.flush();
			LCD.drawInt(_waitcycles,0,6);

		}
	}

	public void buttonPressed(Button b) {}
	   
	public void buttonReleased(Button b) {
		   NXTplatform.exitButton = true;
		   // wait cycle
		   Sound.beepSequence();
		   try{Thread.sleep(2000);}
			catch(InterruptedException e){
				System.exit(0);
			}
		   System.exit(0);
	}
	
}
