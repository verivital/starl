package edu.illinois.mitra.starl.modelinterfaces;


/**
 * iRobot Create specific, contains functions used to control the Create platform.
 * @author Adam Zimmerman
 */
public class BluetoothCommands {
	// Roomba motion commands
	public static byte[] turn(int velocity, int dir) {
		if(dir > 0) {
			return new byte[]{(byte) 137, 0x00, (byte) velocity, (byte) 0xFF, (byte) 0xFF};
		} else {
			return new byte[]{(byte) 137, 0x00, (byte) velocity, 0x00, 0x01};
		}
	}
	
	public static byte[] straight(int velocity) {
		return new byte[]{(byte) 137, 0x00, (byte) velocity, 0x7F, (byte) 0xFF};
	}
	
	public static byte[] curve(int velocity, int radius) {
		return new byte[]{(byte) 137, 0x00, (byte) velocity, (byte) ((radius & 0xFF00) >> 8), (byte) (radius & 0xFF)};
	}
	
	public static byte[] stop() {
		return straight(0);
	}
	
	public static byte[] play_song(int song) {
		return new byte[]{(byte) 141, (byte) song};
	}
	
	//Turns on one of the three LEDs, specified by an int 0->2
	public static byte[] led(int led) {
		byte led_state = (byte) (led!=0?(6*led-4):0x00);
		return new byte[]{(byte) 139, led_state, 0x00, (byte) (led==0?0xFF:0x00)};
	}

	public static byte[] req_sensor(int packetID) {
		return new byte[]{(byte) 142, (byte) packetID};
	}
}
