package edu.illinois.mitra.starl.harness;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.motion.MotionAutomaton_iRobot;
import edu.illinois.mitra.starl.objects.Common;

public class RealisticSimMotionAutomaton_iRobot extends MotionAutomaton_iRobot {
	private SimGpsProvider gpsp;
	private String name;
	private String typeName;
	
	public RealisticSimMotionAutomaton_iRobot(GlobalVarHolder gvh, SimGpsProvider gpsp) {
		super(gvh, null);
		name = gvh.id.getName();
		typeName = "Model_iRobot";
		this.gpsp = gpsp;
	}

	@Override
	public void motion_stop() {
		gpsp.setVelocity(typeName, name, 0, 0);
		super.running = false;
		super.stage = MotionAutomaton_iRobot.STAGE.INIT;
		super.destination = null;
		super.inMotion = false;
	}

	@Override
	protected void curve(int velocity, int radius) {
		if(running) {
			sendMotionEvent(Common.MOT_ARCING, velocity, radius);
			// TODO: Determine if angular velocity formula works! 
			gpsp.setVelocity(typeName,name, velocity, (int) Math.round((velocity*360.0)/(2*Math.PI*radius)));
		}
	}

	@Override
	protected void straight(int velocity) {
		gvh.log.i(TAG, "Straight at velocity " + velocity);
		if(running) {
			if(velocity != 0) {
				sendMotionEvent(Common.MOT_STRAIGHT, velocity);
			} else {
				sendMotionEvent(Common.MOT_STOPPED, 0);
			}
			gpsp.setVelocity(typeName, name, velocity, 0);
		}
	}

	@Override
	protected void turn(int velocity, int angle) {
		if(running) {
			sendMotionEvent(Common.MOT_TURNING, velocity, angle);
			gpsp.setVelocity(typeName, name, 0, (int) Math.copySign(velocity, -angle));
		}
	}	
	

	@Override
	public void cancel() {
		super.running = false;
	}
}
