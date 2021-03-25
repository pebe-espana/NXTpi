import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.Sound;
import lejos.nxt.*;

public class WriteIntegerLog {

	private FileOutputStream out = null;
	private File data = new File("log.dat");
	
	public WriteIntegerLog(){
		
	}
	
	public void Write(int[] intData) {
		
		try {
			out = new FileOutputStream(data);
		} catch(IOException e) {
			System.err.println("Failed to create OS");
			Sound.beep();
			Button.ESCAPE.waitForPress();
			System.exit(1);
		}
		
		DataOutputStream dataOut = new DataOutputStream(out);
		
		try { // to write data
			for (int i = 0; i< intData.length; i++) {
				dataOut.writeInt(intData[i]);
			}
			out.close();
		} catch(IOException e) {
			System.err.println("Failed to write OS");
			Sound.beep();
			Button.ESCAPE.waitForPress();
		}
		
		Sound.beepSequence();
		
	}

	
}
