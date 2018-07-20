package edu.illinois.mitra.starl.interfaces;

/**
 * Objects implementing RobotEventListener may be registered with the GVH to receive system events.
 * Any object with access to the main GVH may send a system event.
 * 
 * @author Adam Zimmerman
 * @version 1.0
 * 
 */
public interface RobotEventListener {
	public static enum Event {MOTION, GPS, GPS_SELF, WAYPOINT_RECEIVED};
	/**
	 * Called by robot event providers when an event occurs
	 * @param eventType the event type
	 * @param eventData (optional) any data associated with the event
	 */
	public void robotEvent(Event eventType, int eventData);
}
