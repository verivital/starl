package edu.illinois.mitra.starl.harness;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.models.Model_Phantom;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Phantom;


public class RealisticSimMotionAutomaton_Phantom extends MotionAutomaton_Phantom {
	private SimGpsProvider gpsp;
	private String name;
	private Model_Phantom my_model;
	private float maxTilt;

	public RealisticSimMotionAutomaton_Phantom(GlobalVarHolder gvh, SimGpsProvider gpsp) {
		super(gvh, null);
		name = gvh.id.getName();
		this.gpsp = gpsp;
		this.my_model = (Model_Phantom)gvh.plat.model;
		maxTilt = (float)this.my_model.max_pitch_roll();
	}

	@Override
	public void setControlInput(double yaw_v, double pitch, double roll, double gaz){
//		double temp;
//		temp = pitch;
//		pitch = roll;
		pitch *= -1;
		roll *= -1;
		if(yaw_v > 1 || yaw_v < -1){
			throw new IllegalArgumentException("yaw speed must be between -1 to 1");
		}
		if(pitch > 1 || pitch < -1){
			throw new IllegalArgumentException("pitch must be between -1 to 1");
		}
		if(roll > 1 || roll < -1){
			throw new IllegalArgumentException("roll speed must be between -1 to 1");
		}
		if(gaz > 1 || gaz < -1){
			throw new IllegalArgumentException("gaz, vertical speed must be between -1 to 1");
		}
		System.out.printf("CINPUT: %f %f %f %f\n", yaw_v, pitch, roll, gaz);
		//gpsp.setControlInputPhantom(name, yaw_v*my_model.max_yaw_speed, pitch*maxTilt, roll*maxTilt, gaz*my_model.max_gaz);
		//gpsp.setControlInputPhantom(name, yaw_v*1500, 0, 0, gaz*my_model.max_gaz);
	}

	/**
	 *  	take off from ground
	 */
	@Override
	protected void setMaxTilt(float val) {
		maxTilt = val;
	}

	@Override
	protected void rotateDrone(){
		setControlInputRescale(calculateYaw(), 0, 0, 0);
	}

	@Override
	protected void takeOff(){
		gvh.log.i(TAG, "Drone taking off");
		setControlInput(0, 0, 0, 1);
	}
	
	/**
	 * land on the ground
	 */
	@Override
	protected void land(){
		gvh.log.i(TAG, "Drone landing");
		//setControlInput(my_model.yaw, 0, 0, 5);
	}
	
	/**
	 * hover at current position
	 */
	@Override
	protected void hover(){
		gvh.log.i(TAG, "Drone hovering");
		setControlInput(0, 0, 0, 0);
	}

	@Override
	public void cancel() {
		super.running = false;
	}
}
