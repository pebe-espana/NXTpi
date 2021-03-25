//package NXT_definitions;

public class Lejos_stringSplit {

/* this class is written for Lejos version of MessageStructure,
 * as LeJos does not have a method <String>.split("SubString")
 * which separates a String into its sub-parts with "SubString" as separator
 */
	
	private String msg = "";
	
	public Lejos_stringSplit(String msg){
		this.msg = msg;
	}
	
	public String[] doSplit(String separator){
		
		String interim[] = new String[100];
		
		int sp_pos = 0;
		int j = 0;
		String remainder;
		
		remainder = this.msg;
		int i1 = 0;
		while (i1 < remainder.length()) {
			//System.out.println(" while ");
			sp_pos = remainder.indexOf(separator,0);
			//System.out.println(remainder + " >" + sp_pos);
			
			if (sp_pos > 0) {
				interim[j] = remainder.substring(0,sp_pos);
				remainder = remainder.substring(sp_pos+1,remainder.length());
				j++;
			}
			//System.out.println(j + " " + interim[j-1]);
			//System.out.println(remainder);
			i1 = sp_pos;
		}
		
		String result[] = new String[j+1];
		
	    for (int i=0; i<j; i++) {
	    	result[i] = interim[i];
	    }
	    result[j] = remainder;
		return result;
	}
	
	// to test the above class
	/*
	
	public static void main(String[] args) {
		String message = "00 1 222 333 4444 55";
		Lejos_stringSplit test = new Lejos_stringSplit(message);
		
		String str[], str2[];
		
		str = message.split(" ");
		str2 = test.doSplit(" ");
		
		if (str.length != str2.length) {
			System.out.println(" something is wrong");
			System.out.println(" Java native method gets :" + str.length);
			System.out.println(" my own method gets :" + str2.length);
			System.out.println(str[0] + ";" + str[1] + ";" + str[2] + ";" + str[3] + ";" + str[4] + ";" + str[5]);
			System.out.println(str2[0] + ";" + str2[1] + ";" + str2[2] + ";" + str2[3] + ";" + str2[4]+ ";" + str2[5]);
		} else {
			System.out.println("found length " + str.length);
			System.out.println(str[0] + ";" + str[1] + ";" + str[2] + ";" + str[3] + ";" + str[4]+ ";" + str[5]);
			System.out.println(str2[0] + ";" + str2[1] + ";" + str2[2]+ ";" + str2[3] + ";" + str2[4]+ ";" + str2[5]);
		}
	}
	
	*/
}
