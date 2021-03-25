
public class TranslateCMD implements CommandConstants{
	
	private int _Ncmd = COMMAND_LENGTH;
	private int _NBcmd = B_COMMAND_LENGTH;
	
	public TranslateCMD() {
		// just instantiate for the use of method
	}
	
	public int[] getCMD(String[] _pycmd) {
		
		int[] cmdint = new int[_Ncmd];
		int raw, dig1, dig2, dig3;
		boolean negative;
		
		//first integer = command constant
		cmdint[0] = Integer.parseInt(_pycmd[0])-48;
		
		//second, third and fourth integer = 3-digit number, requires 4 strings read (incl sign)
		// 0 = 0; dealt with above
		// 1 = 1,2,3,4,5     j=1  k=1,2,3,4,5
		// 2 = 6,7,8,9,10    j=2
		// 3 = 11,12,13,14,15  j=3
		for (int j=1; j<4; j++) {
			raw = Integer.parseInt(_pycmd[5*(j-1)+1]);  // = index 1,6,11
			if (raw == 45) { 
				negative = true;
				dig1 = 0;
			} else {
				negative = false; 
				dig1 = raw - 48;
			}
			raw = Integer.parseInt(_pycmd[5*(j-1)+2]);  
			dig2 = raw - 48;
			for (int k=3; k<6; k++) {
				raw = Integer.parseInt(_pycmd[5*(j-1)+k]);
				cmdint[j] = 10 * dig2 + (raw-48);
				dig2 = cmdint[j];
			}
			if (negative) { cmdint[j] = - cmdint[j]; }
		}
		
		return cmdint;
	}

}
