import lejos.nxt.LCD;
import lejos.nxt.Sound;

public class TranslateREP implements CommandConstants{
	

	private int _NBrep = B_MESSAGE_LENGTH;  // maximum possible length
	private int[] ans_expected = CMD_ANS_N;
	
	public TranslateREP() {
		// just instantiate for the use of method
	}
	
	public int[] getREP(int[] _pyIrep, int for_command, int n_exp) {
		
		int len = 0;
		int digit;
		int n;

		int j = 0; // the index over reply bytes vector
		
		LCD.drawInt(for_command, 0, 7);
		LCD.drawInt(n_exp, 5, 7);
		
		// run index over the (expected answer size <= _Nrep) * 5
		n = 5 * ans_expected[for_command];  // 5 decimal places for each integer including - sign
		if (n != n_exp) {
			// something seriously wrong!
			LCD.clear();
			LCD.drawString(" length error in getRep", 0, 0);
			LCD.drawInt(n_exp, 1, 1);
			LCD.drawInt(n, 1, 2);
			n_exp = 5; // send back only one value!!
			try{Thread.sleep(2000);}
			catch(InterruptedException e){
				System.exit(0);
			}
		}
		if (n > _NBrep) {
			// something seriously wrong!
			LCD.clear();
			LCD.drawString(" length error in getRep", 0, 0);
			LCD.drawInt(n_exp, 1, 1);
			LCD.drawInt(n, 1, 2);
			n_exp = 5; // send back only one value!!
			try{Thread.sleep(2000);}
			catch(InterruptedException e){
				System.exit(0);
			}
		}
		

		
		// now define the reply byte array in terms of ascii integers
		int [] reply = new int[n_exp];
		
		n = n_exp/5;  //loop over n integers for n_exp bytes
		for (int k=0; k<n; k++) {
			// determine size of integer
			if (_pyIrep[k] < 0) {
				// negative = insert minus
				reply[j] = 45;
				_pyIrep[k] = - _pyIrep[k];  // work with positive number
			} else {
				reply[j] = 48;
			}
			j++;
			if (_pyIrep[k] >= 1000) {
				digit = _pyIrep[k] / 1000;
				reply[j] = 48+digit;
				_pyIrep[k] = _pyIrep[k] - (digit * 1000);
			} else {
				reply[j] = 48;
			}
			j++;
			if (_pyIrep[k] >= 100) {
				digit = _pyIrep[k] / 100;
				reply[j] = 48+digit;
				_pyIrep[k] = _pyIrep[k] - (digit * 100);
			} else {
				reply[j] = 48;
			}			
			j++;
			if (_pyIrep[k] >= 10) {
				digit = _pyIrep[k] / 10;
				reply[j] = 48+digit;
				_pyIrep[k] = _pyIrep[k] - (digit * 10);
			} else {
				reply[j] = 48;
			}	
			j++;
			if (_pyIrep[k] >= 1) {
				digit = _pyIrep[k];
				reply[j] = 48+digit;
				_pyIrep[k] = 0;
			} else {
				reply[j] = 48;
			}	
			j++;
		}
		
		if (j != n_exp) {
			LCD.clear();
			LCD.drawString(" length error in getRep", 0, 0);
			LCD.drawInt(n_exp, 1, 1);
			LCD.drawInt(j, 1, 2);
			
			Sound.beepSequenceUp();
			try{Thread.sleep(5000);}
			catch(InterruptedException e){
				System.exit(0);
			}
		}
		return reply;
	}

}

