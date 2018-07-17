package edu.illinois.mitra.starl.gvh;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.illinois.mitra.starl.interfaces.RobotEventListener;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;

/**
 * The GlobalVarHolder encapsulates all communication, location, identification, logging, tracing, and platform specific functionality. It is the core of
 * the StarL framework and is used by nearly all StarL functions.
 * 
 * @author Adam Zimmerman
 * @version 2.0
 */
public abstract class GlobalVarHolder {

	public Comms comms;
	public Gps gps;
	public Id id;
	public Logging log;
	public Trace trace;
	public AndroidPlatform plat;

	// To assign each bot in specific group
	public RobotGroup BotGroup;
	
	/**
	 * @param name The unique identifier of this agent
	 * @param participants A HashMap linking participating agent identifiers to their IP addresses
	 */
	public GlobalVarHolder(String name, Map<String,String> participants) {
		id = new Id(name, participants);
	}
	
	// Events
	private Set<RobotEventListener> eventListeners = new HashSet<RobotEventListener>();

	
	/**
	 * Register a RobotEventListener to receive all future system events
	 * @param el a listener to register
	 * @see RobotEventListener
	 */
	public void addEventListener(RobotEventListener el) {
		eventListeners.add(el);
	}
	/**
	 * Remove a RobotEventListener from the list of registered event receivers
	 * @param el a listener to unregister
	 * @see RobotEventListener
	 */
	public void removeEventListener(RobotEventListener el) {
		eventListeners.remove(el);
	}
	/**
	 * Sends a new event to all registered RobotEventLitseners
	 * @param type the event type
	 * @param event any additional event information
	 */
	public void sendRobotEvent(Event type, int event) {
		for(RobotEventListener el : eventListeners) {
			el.robotEvent(type, event);
		}
	}
	/**
	 * Sends a new event to all registered RobotEventLitseners without including additional event information
	 * @param type the event type
	 */
	public void sendRobotEvent(Event type) {
		sendRobotEvent(type, -1);
	}
	
	/**
	 * Pauses the calling thread 
	 * @param time the number of milliseconds to sleep for
	 */
	public abstract void sleep(long time);
	
	/**
	 * @return the current system time in milliseconds since the Unix Epoch 
	 */
	public abstract long time();
	
	/**
	 * Registers a thread with the simulation framework. In real implementations, this is a stub.
	 * @param thread the thread to register
	 */
	public abstract void threadCreated(Thread thread);
	
	/**
	 * Unregister a thread with the simulation framework. In real implementations, this is a stub.
	 * @param thread the thread to unregister
	 */
	public abstract void threadDestroyed(Thread thread);

}