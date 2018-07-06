package edu.illinois.mitra.starl.motion;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.UUID;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.os.AsyncTask;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.objects.HandlerMessage;

/**
 * Android specific. Maintains a Bluetooth socket connection to the iRobot
 * Create platform.
 * 
 * @author Adam Zimmerman
 * 
 */
public class BluetoothInterface implements GroundBTI {

	private static final String ERR = "Critical Error";
	private static final String TAG = "BluetoothInterface";
	private final static byte[] ENABLE_CONTROL = { (byte) 128, (byte) 132 };
	private final static byte[] PROGRAM_SONG = { (byte) 140, 0x00, 0x02, (byte) 79, (byte) 16, (byte) 84, (byte) 16, (byte) 140, 0x01, 0x01, (byte) 96, (byte) 10, (byte) 140, 0x02, 0x01, (byte) 84, (byte) 10 };
	
	// Standard serial port UUID (from Android documentation)
	private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

	private BluetoothAdapter btAdapter;
	private BluetoothDevice mDevice;
	private BluetoothSocket mSocket;
	private OutputStream outStream;
	private BufferedInputStream inStream;
	
	private GlobalVarHolder gvh;
	private String targetMacAddress;

	private BluetoothConnectTask task;

	private boolean running = true;

	private boolean isConnected = false;

	public BluetoothInterface(GlobalVarHolder gvh, String mac) {
		this.gvh = gvh;
		this.targetMacAddress = mac;

		btAdapter = BluetoothAdapter.getDefaultAdapter();
		if(btAdapter == null)
			throw new RuntimeException("NO BLUETOOTH ADAPTER FOUND!");

		connect();
	}

	public void connect() {
		gvh.log.i(TAG, "Connecting to " + targetMacAddress);
		task = new BluetoothConnectTask();

		if(btAdapter.isDiscovering())
			btAdapter.cancelDiscovery();

		if(!BluetoothAdapter.checkBluetoothAddress(targetMacAddress))
			gvh.log.e(TAG, "Not a valid Bluetooth address!");
		
		// Acquire the remote device
		mDevice = btAdapter.getRemoteDevice(targetMacAddress);

		// Attempt to connect
		task.execute(mDevice);
	}

	public synchronized void send(byte[] to_send) {
		if(outStream != null) {
			try {
				outStream.write(to_send);
			} catch(IOException e) {
				gvh.log.e(ERR, "Bluetooth failed to send!");
			} catch(NullPointerException e) {
				gvh.log.e(ERR, "Bluetooth write failed: mOutStream throws null pointer exception");
			}
		}
	}

	public synchronized byte[] readBuffer(int n_bytes) {
		byte[] buffer = new byte[n_bytes];
		try {
			inStream.read(buffer);
		} catch(IOException e) {
			gvh.log.i(TAG, "Failed to read anything!");
		} catch(NullPointerException e) {
			gvh.log.i(TAG, "We're not connected yet! Go away!");
			return null;
		}

		return buffer;
	}

	public synchronized void clearBuffer() {
		try {
			inStream.skip(inStream.available());
		} catch(IOException e) {
			e.printStackTrace();
		}
	}

	public synchronized byte[] sendReceive(byte[] to_send, int expectedBytes) {
		send(to_send);
		return readBuffer(expectedBytes);
	}

	public void disconnect() {
		running = false;
		gvh.log.e(TAG, "Disconnecting from bluetooth!");
		if(mSocket != null) {
			try {
				mSocket.close();
			} catch(IOException e) {
				gvh.log.e(ERR, "Bluetooth failed to disconnect!");
			}
		}

		try {
			task.cancel(true);
		} catch(Exception e) {
			gvh.log.e(TAG, "Tried to stop BT connect task, failed.");
		}
		outStream = null;
		gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_BLUETOOTH, HandlerMessage.BLUETOOTH_DISCONNECTED);
	}

	private class BluetoothConnectTask extends AsyncTask<BluetoothDevice, Void, Integer> {
		@Override
		protected Integer doInBackground(BluetoothDevice... params) {
			gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_BLUETOOTH, HandlerMessage.BLUETOOTH_CONNECTING);

			BluetoothSocket tmp = null;
			try {
				tmp = params[0].createInsecureRfcommSocketToServiceRecord(MY_UUID);
			} catch(IOException e) {
				gvh.log.e(TAG, "Couldn't create socket!");
			}
			mSocket = tmp;

			try {
				mSocket.connect();
				outStream = mSocket.getOutputStream();
				inStream = new BufferedInputStream(mSocket.getInputStream());
			} catch(IOException e) {
				outStream = null;
				inStream = null;
				gvh.log.e(TAG, "Failed to connect! Retrying...");
				if(running)
					BluetoothInterface.this.connect();
				return 1;
			}

			isConnected = true;
			gvh.log.i(TAG, "Connection established.");
			send(ENABLE_CONTROL);
			send(PROGRAM_SONG);
			try {
				// Clear the buffer
				inStream.read(new byte[inStream.available()], 0, inStream.available());
			} catch(IOException e) {
				e.printStackTrace();
			}
			// Inform the GUI that bluetooth has been connected
			gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_BLUETOOTH, HandlerMessage.BLUETOOTH_CONNECTED);
			return 0;
		}
	}
}