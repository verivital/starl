package edu.illinois.mitra.starl.objects;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * The static Common class is used to store StarL constants used for messaging, event handling, and GUI updates. A series of useful functions
 * are included as well.
 * @author Adam Zimmerman
 * @version 1.0
 */
public final class Common {
	private static final String TAG = "Common";
	private static final String ERR = "Critical Error";

	// Specify the number of robots and number of groups

	public static final Integer numOFbots = 5;
	public static final Integer numOFgroups= 2;

    // Mobile device types
    public static final int NEXUS7 = 60;
    public static final int MOTOE = 61;


	// Message IDs
	public static final int MSG_BARRIERSYNC 			= 1;
	public static final int MSG_MUTEX_TOKEN_OWNER_BCAST = 2;
	public static final int MSG_MUTEX_TOKEN 			= 3;
	public static final int MSG_MUTEX_TOKEN_REQUEST 	= 4;
	public static final int MSG_RANDLEADERELECT 		= 5;
	public static final int MSG_RANDLEADERELECT_ANNOUNCE= 6;
	public static final int MSG_NETWORK_DISCOVERY		= 7;
	public static final int MSG_BULLYELECTION 			= 8;
	public static final int MSG_BULLYANSWER 			= 9;
	public static final int MSG_BULLYWINNER				= 10;
	public static final int MSG_ACTIVITYLAUNCH			= 11;
	public static final int MSG_ACTIVITYABORT			= 12;
	public static final int MSG_GEOCAST					= 13;
	public static final int MSG_BOARDCAST               = 14;
	public static final int MSG_DSM_INFORM              = 15;
	//reserver 1000*2 mutex id for the group set mutex algorithm
	public static final int MSG_GSMUTEX_REQUEST         = 1000;
	public static final int MSG_GSMUTEX_REPLY           = 2000;
	// GUI Message handler
//	public static final int MESSAGE_TOAST = 0;
//	public static final int MESSAGE_LOCATION = 1;
//	public static final int MESSAGE_BLUETOOTH = 2;
//	public static final int MESSAGE_LAUNCH = 3;
//	public static final int MESSAGE_ABORT = 4;
//	public static final int MESSAGE_DEBUG = 5;
//	public static final int MESSAGE_BATTERY = 6;
//	
//	public static final int BLUETOOTH_CONNECTING = 2;
//	public static final int BLUETOOTH_CONNECTED = 1;
//	public static final int BLUETOOTH_DISCONNECTED = 1;
//	public static final int GPS_RECEIVING = 1;
//	public static final int GPS_OFFLINE = 0;
	
	// Motion types
	public static final int MOT_TURNING		= 0;
	public static final int MOT_ARCING		= 1;
	public static final int MOT_STRAIGHT	= 2;
	public static final int MOT_STOPPED		= 3;

	// Event types
	public static final int EVENT_MOTION = 0;
	public static final int EVENT_GPS = 1;
	public static final int EVENT_GPS_SELF = 2;
	public static final int EVENT_WAYPOINT_RECEIVED = 3;


	
	private Common() {
	}
	
	/**
	 * Convert an array of strings to an array of integers. Returns null if any array element doesn't contain a number
	 * @param parts an array integers in string form
	 * @return an array of integers parsed from the array of strings.
	 */
	public static int[] partsToInts(String[] parts) {
		int[] retval = new int[parts.length];
		for(int i = 0; i < parts.length; i++) {
			try {
				retval[i] = Integer.parseInt(parts[i]);
			} catch(NumberFormatException e) {
				//Log.e(TAG, "Can't parse " + parts[i] + " as an integer!");
				return null;
			}
		}
		return retval;
	}
	
	/**
	 * @param a1
	 * @param a2
	 * @return The input parameter with the smallest magnitude
	 */
	public static int min_magitude(int a1, int a2) {
		if(Math.abs(a1) < Math.abs(a2)) {
			return a1;
		} else {
			return a2;
		}
	}
	
	/**
	 * Convert a series of integers to a string array
	 * @param pieces
	 * @return String representation of the input integers
	 */
	public static String[] intsToStrings(Integer ... pieces) {
		String[] retval = new String[pieces.length];
		for(int i = 0; i < pieces.length; i++) {
			retval[i] = pieces[i].toString();
		}
		return retval;
	}
	
	/**
	 * @param str The input string to process
	 * @param delimiter The delimiting string used to split the input string
	 * @return An array contining the integers parsed from the input string
	 */
	public static int[] partsToInts(String str, String delimiter) {
		String[] parts = str.split(delimiter);
		return partsToInts(parts);
	}
	
	// Common value manipulation and comparison functions
	/**
	 * @param val value to test 
	 * @param min minimum acceptable value
	 * @param max maximum acceptable value
	 * @return true if val is between min and max, false otherwise
	 */
	public static <T extends Comparable<T>> boolean inRange(T val, T min, T max) {
		if(val.compareTo(min) >= 0 && val.compareTo(max) <= 0) return true;
		return false;
	}
	
	/**
	 * @param val value to test
	 * @param max maximum acceptable value
	 * @return val if val is less than max, max otherwise
	 */
	public static <T extends Comparable<T>> T cap(T val, T max) {
		if(val.compareTo(max) < 0) {
			return val;
		} else {
			return max;
		}
	}
	
	/**
	 * @param val value to test
	 * @param min minimum acceptable value
	 * @param max maximum acceptable value
	 * @return the value of val, capped between min and max
	 */
	public static <T extends Comparable<T>> T  cap(T val, T min, T max) {
		if(val.compareTo(max) > 0) {
			return max;
		} else if(val.compareTo(min) < 0) {
			return min;
		} else {
			return val;
		}
	}

	public static double cap(double val, double min, double max) {
		// limits the absolute value of 'value' to center plus or minus absLimit, if absLimit is positive
		if (val > max) {
			return max;
		}
		if (val < min) {
			return min;
		}
		return val;
	}
	
    public static InetAddress getLocalAddress()throws IOException {
		try {
		    for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();) {
		        NetworkInterface intf = en.nextElement();
		        for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();) {
		            InetAddress inetAddress = enumIpAddr.nextElement();
		            if (!inetAddress.isLoopbackAddress()) {
		            	return inetAddress;
		            }
		        }
		    }
		} catch (SocketException ex) {
		    //Log.e(TAG, ex.toString());
		}
		return null;
    }
    
	/**
	 * Converts a two byte array to an integer
	 * @param b a byte array of length 2
	 * @return an int representing the unsigned short
	 */
	public static int unsignedShortToInt(byte[] b)
	{
		if(b.length != 2){
			return -99;
		}
	    int i = 0;
	    i |= b[0] & 0xFF;
	    i <<= 8;
	    i |= b[1] & 0xFF;
	    return i;
	}
	
	public static int signedShortToInt(byte[] b)
	{
		if(b.length != 2) {
			return -99;
		}
		int i = ((b[0] & 0xFF) << 8) | (b[1] & 0xFF);
//		i |= b[0];
//		i <<= 8;
//		i |= b[1];
		return i;
	}

	
	/**
	 * Converts an input value to an angle between 0 and 359 degrees (360 degree range)
	 * @param angle the angle to be rectified
	 * @return a rectified angle value
	 */
	public static double angleWrap(double angle) {
		angle = angle % 360;
		return angle < 0 ? angle + 360 : angle;
	}


	// Following array represents the neighbours for each bot and rf distance between them. Index represents the bot num. As following:
	// bots_neighbour [bot_number][0= beforeBot_name], bots_neighbour [bot_number][1= afterBot_name], bots_neighbour [bot_number][2= rf]
	public static String[][] bots_neighbour = new String[numOFbots][3];
}
