package edu.illinois.mitra.starl.comms;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.Vector;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.GpsReceiver;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.ModelRegistry;
import edu.illinois.mitra.starl.models.Model_Phantom;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.HandlerMessage;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * Hardware specific. Opens a UDP socket for receiving GPS broadcasts. 
 * @author Adam Zimmerman
 * @version 1.0
 */
public class UdpGpsReceiver extends Thread implements GpsReceiver {
	private static final String TAG = "GPSReceiver";
	private static final String ERR = "Critical Error";
	
	public PositionList<ItemPosition> robotPositions;
	public PositionList<ItemPosition> waypointPositions;
	public ObstacleList obs;
	public Vector<ObstacleList> viewsOfWorld;
	

	private GlobalVarHolder gvh;

	private DatagramSocket mSocket;
	private InetAddress myLocalIP;
	private boolean running = true;
	private String name = null;
	private boolean received = false;

	public UdpGpsReceiver(GlobalVarHolder gvh,String hostname, int port, PositionList robotPositions, PositionList<ItemPosition> waypointPositions, ObstacleList obs, Vector<ObstacleList> viewsOfWorld) {
		super();
		this.gvh = gvh;
		
		name = gvh.id.getName();
		this.robotPositions = robotPositions;
		this.waypointPositions = waypointPositions;
		this.obs = obs;
		this.viewsOfWorld = viewsOfWorld;

		try {
			myLocalIP = getLocalAddress();
			//changed from:
			//mSocket = new DatagramSocket(port);
			//fix java.lang.NullPointerException: Attempt to invoke virtual method 'void java.net.DatagramSocket.receive(java.net.DatagramPacket)' on a null object reference
			mSocket = new DatagramSocket(null);
			mSocket.setReuseAddress(true);
			mSocket.bind(new InetSocketAddress(port));
		} catch (IOException e) {
			gvh.log.e(TAG, "Unable to create socket!");
			e.printStackTrace();
		}
		
		gvh.log.i(TAG, "Listening to GPS host on port " + port);
		gvh.trace.traceEvent(TAG, "Created", gvh.time());
	}

	@Override
	public synchronized void start() {
		gvh.log.i("GPSReceiver", "Starting GPS receiver");
		running = true;
		super.start();
	}

	@Override
	public void run() {
    		byte[] buf = new byte[2048]; 
    		
    		while(running) {
    	    	try {
		    	// Receive a message
    			DatagramPacket packet = new DatagramPacket(buf, buf.length); 
				mSocket.receive(packet);
    			InetAddress remoteIP = packet.getAddress();
    			if(remoteIP.equals(myLocalIP))
    				continue;

    			String line = new String(packet.getData(), 0, packet.getLength());
    		
    			// Parse the received string
    			String [] parts = line.split("\n");
    			if(received == false) {
    				gvh.log.i(TAG, "RECEIVED FIRST PACKET!");
    				gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_LOCATION, HandlerMessage.GPS_RECEIVING);
    				received = true;
    			}    			
    			for(String part : parts) {
    				if(part.length() >= 2) {
		    			switch(part.charAt(0)) {
		    			case '@':
		    				try {
		    					ItemPosition newpos = new ItemPosition(part);
		    					waypointPositions.update(newpos, gvh.time());
		    					gvh.trace.traceEvent(TAG, "Received Waypoint", newpos, gvh.time());
		    				} catch(ItemFormattingException e){
		    					gvh.log.e(TAG, "Invalid item formatting: " + e.getError());
		    				}
		    				break;

		    			case '#':
		    				try {
		    					//Model_iRobot newpos = new Model_iRobot(parts[i]);
								Model newpos = ModelRegistry.create(Model_iRobot.class.getSimpleName(), part);
		    					robotPositions.update(newpos, gvh.time());
		    					gvh.sendRobotEvent(Event.GPS);
		    					if(newpos.name.equals(name)) {
		    						gvh.trace.traceEvent(TAG, "Received Position", newpos, gvh.time());
		    						gvh.sendRobotEvent(Event.GPS_SELF);
		    					}
		    				} catch(RuntimeException e){
		    					gvh.log.e(TAG, "Invalid item formatting: " + e);
		    				}
		    				break;
		    			case '$':
		    				try {
		    					Model_Phantom newpos = new Model_Phantom(part); //TODO(TIM) MAKE THIS APPLY TO ALL ROBOTS SUPER IMPORTANT
		    					robotPositions.update(newpos, gvh.time());
		    					gvh.sendRobotEvent(Event.GPS);
		    					if(newpos.name.equals(name)) {
		    						gvh.trace.traceEvent(TAG, "Received Position", newpos, gvh.time());
		    						gvh.sendRobotEvent(Event.GPS_SELF);
		    					}
		    				} catch(ItemFormattingException e){
		    					gvh.log.e(TAG, "Invalid item formatting: " + e.getError());
		    				}
		    				break;
		    			case 'G':
		    				gvh.trace.traceEvent(TAG, "Received launch command", gvh.time());
		    				int[] args = Common.partsToInts(part.substring(3).split(" "));
		    				gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_LAUNCH, args[0], args[1]);
		    				break;
		    			case 'A':
		    				gvh.trace.traceEvent(TAG, "Received abort command", gvh.time());
		    				gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_ABORT, null);
		    				break;
		    			default:
		    				gvh.log.e(ERR, "Unknown GPS message received: " + line);
		    				break;
		    			}
    				}
    			}
			} catch (IOException e) {
				gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_LOCATION, HandlerMessage.GPS_OFFLINE);
			}
    	}
	}
	
    private InetAddress getLocalAddress() throws IOException {
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
		    gvh.log.e(TAG, ex.toString());
		}
		return null;
    }
    
    /* (non-Javadoc)
	 * @see edu.illinois.mitra.starl.comms.GpsReceiver#cancel()
	 */
    @Override
    public void cancel() {
    	running = false;
    	gvh.plat.sendMainMsg(HandlerMessage.MESSAGE_LOCATION, HandlerMessage.GPS_OFFLINE);
        try {
        	mSocket.disconnect();
            mSocket.close();
        } catch (Exception e) {
            gvh.log.e(ERR, "close of connect socket failed" + e);
            e.printStackTrace();
        }
        gvh.log.i(TAG, "Closed UDP GPS socket");
		gvh.trace.traceEvent(TAG, "Cancelled", gvh.time());
    }

	@Override
	public PositionList<ItemPosition> getWaypoints() {
		return waypointPositions;
	}
	
	@Override
	public ObstacleList getObspoints() {
		return obs;
	}

	@Override
	public Vector<ObstacleList> getViews() {
		return viewsOfWorld;
	}

	@Override
	public PositionList<ItemPosition> getSensepoints() {
		// TODO work in progress, sensepoints data should really come in as the robot's sensor data, this should be more general to accommodate any sensor data
		return null;
	}

	@Override
	public PositionList<ItemPosition> get_robots() {
		return robotPositions;
	}
	
	
	
}