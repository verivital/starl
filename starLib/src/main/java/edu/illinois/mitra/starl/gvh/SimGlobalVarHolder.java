package edu.illinois.mitra.starl.gvh;

import java.util.HashMap;

import edu.illinois.mitra.starl.harness.RealisticSimMotionAutomaton_Drone;
import edu.illinois.mitra.starl.harness.RealisticSimMotionAutomaton_Ground;
import edu.illinois.mitra.starl.harness.SimGpsReceiver;
import edu.illinois.mitra.starl.harness.SimSmartComThread;
import edu.illinois.mitra.starl.harness.SimulationEngine;
import edu.illinois.mitra.starl.models.*;
import edu.illinois.mitra.starl.motion.ReachAvoid;

/**
 * Extension of the GlobalVarHolder class for use in simulations of StarL applications 
 * @author Adam Zimmerman, Yixiao Lin
 * @version 2.0
 *
 */
public class SimGlobalVarHolder extends GlobalVarHolder {

	private static final String TAG = "SimGlobalVarHolder";
	
	private SimulationEngine engine;
	
	/**
	 * @param name the name of this agent
	 * @param participants contains (name,IP) pairs for each participating agent 
	 * @param engine the main SimulationEngine
	 * @param initpos this agent's initial position
	 * @param traceDir the directory to write trace files to
	 */
	public SimGlobalVarHolder(String name, HashMap<String,String> participants, SimulationEngine engine, Model initpos, String traceDir, int trace_driftMax, double trace_skewBound) {
		super(name, participants);
		this.engine = engine;
		super.comms = new Comms(this, new SimSmartComThread(this, engine.getComChannel()));
		super.gps = new Gps(this, new SimGpsReceiver(this, engine.getGps(), initpos));
		super.log = new SimLogging(name,this);
		super.trace = new Trace(name, traceDir, this);
		if(traceDir != null)
			trace.traceStart();
		super.plat = new AndroidPlatform();
		plat.model = initpos;
		plat.reachAvoid = new ReachAvoid(this);


		// Model_Drone is base class for all aerial robots.
		// Model_Ground is base class for all ground robots
		if(initpos instanceof Model_Ground){
			plat.moat = new RealisticSimMotionAutomaton_Ground(this, engine.getGps());
		} else if(initpos instanceof Model_Drone){
			plat.moat = new RealisticSimMotionAutomaton_Drone(this, engine.getGps());
		} else {
			throw new RuntimeException("Initpos neither a Model_Ground or Model_Drone: " + initpos.getTypeName());
		}
		plat.moat.start();
	}

	@Override
	public void sleep(long time) {
		if(time <= 0) throw new RuntimeException("What are you doing?? You can't sleep for <= 0!");
		try {
			engine.threadSleep(time, Thread.currentThread());
			Thread.sleep(Long.MAX_VALUE);
		} catch (InterruptedException e) {
		}
	}

	@Override
	public long time() {
		return engine.getTime();
	}
	
	@Override
	public void threadCreated(Thread thread) {
		engine.registerThread(thread);
	}

	@Override
	public void threadDestroyed(Thread thread) {
		engine.removeThread(thread);
	}	
}