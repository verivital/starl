package edu.illinois.mitra.starl.functions;

import java.util.List;

import edu.illinois.mitra.starl.comms.MessageContents;
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.MessageListener;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;

/**
 * Sends and receives messages with a geographic destination instead of an agential (invented word) destination.
 *  
 * @author Adam Zimmerman
 * @version 1.0
 */
public class Geocaster implements MessageListener {
	private static final String TAG = "Geocaster";
	private GlobalVarHolder gvh;
	
	public Geocaster(GlobalVarHolder gvh) {
		this.gvh = gvh;
		gvh.comms.addMsgListener(this, Common.MSG_GEOCAST);
	}
	
	// Send a message with ID = MID and contents = msgcontents to all robots contained within the rectangle defined by x, y, width, and height
	
	/**
	 * Send a geocast message to all agents within a rectangular area 
	 * @param msgcontents the contents of the message to send
	 * @param MID the ID of the message to send
	 * @param x x origin of target rectangle
	 * @param y y origin of target rectangle
	 * @param width target rectangle width
	 * @param height target rectangle height
	 */
	public void sendGeocast(MessageContents msgcontents, int MID, int x, int y, int width, int height) {
		MessageContents geocastContents = new MessageContents("RECT", Integer.toString(x),Integer.toString(y),Integer.toString(width),Integer.toString(height),Integer.toString(MID));
		geocastContents.append(msgcontents);
		RobotMessage toSend = new RobotMessage("ALL", gvh.id.getName(), Common.MSG_GEOCAST, geocastContents);
		gvh.comms.addOutgoingMessage(toSend);
		gvh.trace.traceEvent(TAG, "Sent Geocast", toSend, gvh.time());
	}
	
	/**
	 * Send a geocast message to all agents within a circular area
	 * @param msgcontents the contents of the message to send
	 * @param MID the ID of the message to send
	 * @param x the x center of the target circle
	 * @param y the y center of the target circle
	 * @param radius the radius of the target circle
	 */
	public void sendGeocast(MessageContents msgcontents, int MID, int x, int y, int radius) {
		MessageContents geocastContents = new MessageContents("CIRCLE", Integer.toString(x),Integer.toString(y),Integer.toString(radius),Integer.toString(MID));
		geocastContents.append(msgcontents);
		RobotMessage toSend = new RobotMessage("ALL", gvh.id.getName(), Common.MSG_GEOCAST, geocastContents);
		gvh.comms.addOutgoingMessage(toSend);	
		gvh.trace.traceEvent(TAG, "Sent Geocast", toSend, gvh.time());
	}

	@Override
	public void messageReceived(RobotMessage m) {
		List<String> contents = m.getContentsList();
		
		String type = contents.get(0);
		
		if(type.equals("RECT")) {
			int minX = Integer.parseInt(contents.get(1));
			int minY = Integer.parseInt(contents.get(2));
			int maxX = minX + Integer.parseInt(contents.get(3));
			int maxY = minX + Integer.parseInt(contents.get(4));
			
			ItemPosition mypos = gvh.gps.getMyPosition();
			
			if(Common.inRange(mypos.x(), minX, maxX) && Common.inRange(mypos.y(), minY, maxY)) {
				int MID = Integer.parseInt(contents.get(5));
				MessageContents receiveContents = new MessageContents();
				receiveContents.append(contents.subList(6, contents.size()));
				gvh.trace.traceEvent(TAG, "Received Geocast", m, gvh.time());
				gvh.comms.addIncomingMessage(new RobotMessage("ALL", m.getFrom(), MID, receiveContents));
			}
		} else if(type.equals("CIRCLE")) {
			int x = Integer.parseInt(contents.get(1));
			int y = Integer.parseInt(contents.get(2));
			int radius = Integer.parseInt(contents.get(3));
			
			ItemPosition mypos = gvh.gps.getMyPosition();
			
			if(mypos.distanceTo(new ItemPosition("t",x,y,0)) <= radius) {
				int MID = Integer.parseInt(contents.get(4));
				MessageContents receiveContents = new MessageContents();
				receiveContents.append(contents.subList(5, contents.size()));
				gvh.trace.traceEvent(TAG, "Received Geocast", m, gvh.time());
				gvh.comms.addIncomingMessage(new RobotMessage("ALL", m.getFrom(), MID, receiveContents));
			}
		}
	}
}
