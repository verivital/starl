package edu.illinois.mitra.starl.motion;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.harness.SimGpsProvider;
import edu.illinois.mitra.starl.models.ModelRegistry;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Ground;
import edu.illinois.mitra.starl.objects.Common;

public class SimMotionAutomaton_Ground extends MotionAutomaton_Ground {
	private SimGpsProvider gpsp;
	private String name;
	private String typeName;
	
	public SimMotionAutomaton_Ground(GlobalVarHolder gvh, SimGpsProvider gpsp) {
		super(gvh);
		name = gvh.id.getName();
		typeName = gvh.plat.model.getTypeName();
		if (!(gvh.plat.model instanceof Model_Ground)) {
			throw new IllegalArgumentException(typeName + " is not an instance of Model_Ground.");
		}
		this.gpsp = gpsp;
	}

	@Override
	public void motion_stop() {
		gpsp.setVelocity(typeName, name, 0, 0);
		super.running = false;
		super.stage = STAGE.INIT;
		super.destination = null;
		super.inMotion = false;
	}

	@Override
	protected void curve(int velocity, int radius) {
		if(running) {
			sendMotionEvent(Common.MOT_ARCING, velocity, radius);
			// TODO: Determine if angular velocity formula works!
			gpsp.setVelocity(typeName, name, velocity, (int) Math.round((velocity*360.0)/(2*Math.PI*radius)));
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
