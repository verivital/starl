package edu.illinois.mitra.starl.gvh;

import java.util.Map;
import java.util.Vector;

import android.content.Context;
import android.os.Handler;
import edu.illinois.mitra.starl.comms.SmartUdpComThread;
import edu.illinois.mitra.starl.comms.UdpGpsReceiver;
import edu.illinois.mitra.starl.harness.RealisticMotionAutomaton_Drone;
import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.motion.BTI;
import edu.illinois.mitra.starl.motion.BluetoothInterface;
import edu.illinois.mitra.starl.motion.DjiController;
import edu.illinois.mitra.starl.motion.GhostAerialBTI;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Drone;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Ground;
import edu.illinois.mitra.starl.motion.ReachAvoid;
import edu.illinois.mitra.starl.motion.MiniDroneBTI;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * Extension of the GlobalVarHolder class for use in physical implementations of StarL applications
 * @author Adam Zimmerman
 * @version 1.0
 */
public class RealGlobalVarHolder extends GlobalVarHolder {

	/**
	 * @param name the name of this agent
	 * @param participants contains (name,IP) pairs for each participating agent
	 * @param handler the main application handler capable of receiving GUI update messages
	 * @param robotMac the MAC address of this agent's iRobot Create chassis
	 */
	public RealGlobalVarHolder(String name, Map<String,String> participants, Model initpos, Handler handler, String robotMac, Context context) {
//	public RealGlobalVarHolder(String name, Map<String,String> participants, Handler handler, String robotMac, Context context) {
		super(name, participants);

		super.log = new AndroidLogging();
		super.trace = new Trace(name, "/sdcard/trace/", this);
		super.plat = new RealAndroidPlatform(handler);
		super.comms = new Comms(this, new SmartUdpComThread(this));
		//super.gps = new Gps(this, new UdpGpsReceiver(this,"192.168.1.100",4000,new PositionList(),new PositionList(), new ObstacleList(), new Vector<ObstacleList>(3,2) ));
		super.gps = new Gps(this, new UdpGpsReceiver(this,"10.255.24.100",4000,new PositionList(),new PositionList<>(), new ObstacleList(), new Vector<ObstacleList>(3,2) ));
		plat.model = initpos;
		plat.reachAvoid = new ReachAvoid(this);

		BTI bti; // bluetooth interface
		try {
			bti = plat.model.getBluetoothInterface().newInstance();
		} catch (InstantiationException | IllegalAccessException e) {
			throw new IllegalArgumentException("Could not access bluetooth interface. " + e);
		}

		if (initpos instanceof Model_Drone) {
			plat.moat = new RealisticMotionAutomaton_Drone(this, bti);
		} else if (initpos instanceof Model_Ground) {
			plat.moat = new MotionAutomaton_Ground(this, bti);
		} else {
			throw new IllegalArgumentException("No known MotionAutomaton for type " + plat.model.getTypeName());
		}
		/*
		if(initpos instanceof Model_iRobot) {
			plat.moat = new MotionAutomaton_iRobot(this, new BluetoothInterface(this, robotMac.trim()));
		}

		else if (initpos instanceof Model_quadcopter) {
			plat.moat = new MotionAutomatonMiniDrone(this, new MiniDroneBTI(this, context, robotMac));
		}
		else if(initpos instanceof Model_Mavic){
			plat.moat = new MotionAutomaton_Mavic(this, new DjiController(this, context, robotMac));
		}
		else if(initpos instanceof Model_Phantom){
			plat.moat = new MotionAutomaton_Phantom(this, new DjiController(this, context, robotMac));
		}
		else if (initpos instanceof Model_GhostAerial){
			plat.moat = new MotionAutomaton_GhostAerial(this, new GhostAerialBTI(this, context, robotMac));
		}
		*/
/*
//TD_NATHAN: resolve - resolved above
        if(type == Common.IROBOT) {
            plat.moat = new MotionAutomaton(this, new BluetoothInterface(this, robotMac.trim()));
        }
        else if(type == Common.MINIDRONE) {
            plat.moat = new MotionAutomatonMiniDrone(this, new MiniDroneBTI(this, context, robotMac));
        }
*/
		plat.moat.start();
	}

	@Override
	public void sleep(long time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
		}
	}

	@Override
	public long time() {
		return System.currentTimeMillis();
	}

	@Override
	public void threadCreated(Thread thread) {
		// Nothing happens here
	}

	@Override
	public void threadDestroyed(Thread thread) {
		// Nothing happens here
	}
}
