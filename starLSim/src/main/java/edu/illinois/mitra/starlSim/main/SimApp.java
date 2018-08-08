package edu.illinois.mitra.starlSim.main;

import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.Callable;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.gvh.SimGlobalVarHolder;
import edu.illinois.mitra.starl.harness.SimulationEngine;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.motion.RobotMotion;
import edu.illinois.mitra.starlSim.draw.DrawFrame;

public class SimApp implements Callable<List<Object>> {
	protected String name;
	protected GlobalVarHolder gvh;
	public LogicThread logic;
	public RobotMotion rob;

	public SimApp(String name, HashMap<String, String> participants, SimulationEngine engine, Model initpos, String traceDir, Class<? extends LogicThread> app, DrawFrame drawFrame, int driftMax, double skewBound) {
		this.name = name;
		gvh = new SimGlobalVarHolder(name, participants, engine, initpos, traceDir, driftMax, skewBound);
		gvh.comms.startComms();
		gvh.gps.startGps();




		// Create the class to be simulated
		try {
			// Generically instantiate an instance of the requested LogicThread
			logic = app.getConstructor(GlobalVarHolder.class).newInstance(gvh);
			rob = gvh.plat.moat;
			if(drawFrame != null){
				drawFrame.addPointInputAccepter(logic);		//Adds PointInputAccepter to the logic thread which all Apps extend.
				drawFrame.addKeyInputAccepter(rob);			//Adds keyInputAccepter to the RobotMotion class, which MotionAutomaton extends.

			}
		} catch(InstantiationException e) {
			e.printStackTrace();
		} catch(IllegalAccessException e) {
			e.printStackTrace();
		} catch(IllegalArgumentException e) {
			e.printStackTrace();
		} catch(InvocationTargetException e) {
			e.printStackTrace();
		} catch(NoSuchMethodException e) {
			e.printStackTrace();
		} catch(SecurityException e) {
			e.printStackTrace();
		}
		if(logic == null)
			throw new RuntimeException("Failed to create LogicThread in SimApp class.");
	}

	public String getLog() {
		return gvh.log.getLog();
	}

	@Override
	public List<Object> call() {
		// print exceptions explicitly instead of silently ignoring them
		List<Object> rv = null;

		try {
			rv = logic.call();
		} catch(Exception e) {
			e.printStackTrace();
		}

		return rv;
	}
}