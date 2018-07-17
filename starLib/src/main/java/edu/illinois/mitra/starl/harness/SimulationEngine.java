package edu.illinois.mitra.starl.harness;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.Vector;

import edu.illinois.mitra.starl.gvh.Logging;
import edu.illinois.mitra.starl.interfaces.ExplicitlyDrawable;
import edu.illinois.mitra.starl.interfaces.LogicThread;

/**
 * The core of the simulation. You really don't need to mess with this code.
 * Please don't change anything in here.
 * 
 * @author Adam Zimmerman
 * @version One million
 */
public class SimulationEngine extends Thread {

	private static final int THREAD_DEADLOCK_TIMEOUT = 10000;

	private Map<Thread, Long> threadSleeps = new HashMap<Thread, Long>();
	private Map<Thread, Long> lastUpdateTime = new HashMap<Thread, Long>();

	private Vector<Logging> logs = new Vector<Logging>();
	private SimGpsProvider gps;
	private DecoupledSimComChannel comms;
	private long startTime;
	private long time = 0;
	private long timeout = 0;
	private Object lock = new Object();
	private boolean done = false;
	private double ticRate = 0;

	private double lastTicAdvance = -1;
	private long lastTimeAdvance = -1;

	// for drawing the simulation
	ExplicitlyDrawable drawer = null;
	List<LogicThread> logicThreads = null;

	public SimulationEngine(long timeout, int meanDelay, int delayStdDev, int dropRate, int seed, double ticRate, Set<String> blockedRobots, Map<String, String> nameToIpMap, ExplicitlyDrawable drawer, List<LogicThread> logicThreads) {
		super("SimulationEngine");
		comms = new DecoupledSimComChannel(meanDelay, delayStdDev, dropRate, seed, blockedRobots, nameToIpMap);
		time = System.currentTimeMillis();
		startTime = time;
		done = false;
		this.ticRate = ticRate;
		this.drawer = drawer;
		this.logicThreads = logicThreads;
		this.timeout = startTime + timeout;
		this.start();
	}

	public void threadSleep(long time, Thread thread) {
		synchronized(lock) {
			if(!threadSleeps.containsKey(thread)) {
				throw new RuntimeException("Unregistered thread " + thread + " attempted to sleep for " + time);
			}
			threadSleeps.put(thread, time);
			lastUpdateTime.put(thread, System.currentTimeMillis());

			if(!this.isInterrupted())
				this.interrupt();
		}
	}

	@Override
	public void run() {
		while(!done) {
			try {
				Thread.sleep(5000);
			} catch(InterruptedException e) {
			}

			synchronized(lock) {
				if(clearToAdvance())
					advance();
			}
		}
	}

	private void maintainRealTime() {
		if(ticRate <= 0)
			return;
		if(lastTicAdvance <= 0)
			return;

		// Determine the rate of advance in tics/millisecond
		double rate = lastTicAdvance / (System.currentTimeMillis() - lastTimeAdvance);

		// While the rate is too large, sleep the thread
		while(rate > ticRate) {
			try {
				Thread.sleep(1);
			} catch(InterruptedException e) {
			}
			rate = lastTicAdvance / (System.currentTimeMillis() - lastTimeAdvance);
		}
	}

	private void deadlockCheck(Entry<Thread, Long> entry) {
		long now = System.currentTimeMillis();
		Thread thread = entry.getKey();
		if((entry.getValue() == null) && (now - lastUpdateTime.get(thread)) > THREAD_DEADLOCK_TIMEOUT/ticRate) {

			System.err.println("\n\nPossible deadlock encountered at " + now);

			System.err.println(thread.getId() + " - " + thread.getName());
			StackTraceElement[] st = thread.getStackTrace();
			for(StackTraceElement ste : st) {
				System.err.println(ste.toString());
			}
		}
	}

	private void advance() {
		// Determine if a pause is needed to maintain ties to real-time
		maintainRealTime();

		long advance = comms.minDelay();

		for(Long l : threadSleeps.values()) {
			advance = Math.min(l, advance);
		}

		// force a redraw now of every logic thread
		if(drawer != null){
			drawer.drawNow(logicThreads);
		}
		// Advance time
		time += advance;

		if(timeout != startTime && time > timeout) {
			System.err.println("Simulation timed out! Aborting.");
			simulationDone();
			return;
		}

		comms.advanceTime(advance);

		lastTicAdvance = advance;
		lastTimeAdvance = System.currentTimeMillis();

		// Detect threads to be woken
		for(Entry<Thread, Long> e : threadSleeps.entrySet()) {
			e.setValue(e.getValue() - advance);

			if(e.getValue() == 0) {
				e.setValue(null);
				e.getKey().interrupt();
			}
		}
	}

	public void registerThread(Thread thread) {
		synchronized(lock) {
			lastUpdateTime.put(thread, System.currentTimeMillis());
			threadSleeps.put(thread, null);
		}
	}

	public void removeThread(Thread thread) {
		synchronized(lock) {
			if(!threadSleeps.containsKey(thread))
				throw new RuntimeException("Thread " + thread + " tried to unregister itself without being registered! What a jerk.");

			threadSleeps.remove(thread);
			lastUpdateTime.remove(thread);
		}
		if(!this.isInterrupted())
			this.interrupt();
	}

	public void simulationDone() {
		for(int i = 0; i< logs.size(); i++){
			logs.get(i).saveLogFile();
		}
		done = true;
	}

	private boolean clearToAdvance() {
		for(Entry<Thread, Long> entry : threadSleeps.entrySet()) {
			if(entry.getValue() == null) {
				deadlockCheck(entry);
				return false;
			}
		}
		return true;
	}

	public Long getTime() {
		return time;
	}

	public Long getDuration() {
		return (time - startTime);
	}

	public SimGpsProvider getGps() {
		return gps;
	}

	public void setGps(SimGpsProvider gps) {
		this.gps = gps;
	}
	
	public void addLogging(Logging log){
		this.logs.addElement(log); 
	}

	public DecoupledSimComChannel getComChannel() {
		return comms;
	}
}
