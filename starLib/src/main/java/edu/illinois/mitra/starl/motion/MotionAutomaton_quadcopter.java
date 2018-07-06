package edu.illinois.mitra.starl.motion;

import java.util.*;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.objects.*;

/**
 * This motion controller is for quadcopter models only
 *
 * Motion controller which extends the RobotMotion abstract class. Capable of
 * going to destination or passing through a destination without stopping.
 * Includes optional collision avoidance which is controlled
 * by the motion parameters setting.
 *
 * @author Yixiao Lin
 * @version 1.0
 */
public class MotionAutomaton_quadcopter extends RobotMotion {
	protected static final String TAG = "MotionAutomaton";
	protected static final String ERR = "Critical Error";
	final int safeHeight = 500;

	protected GlobalVarHolder gvh;
	protected BluetoothInterface bti;

	// Motion tracking
	protected ItemPosition destination;
	private Model_quadcopter mypos;


	protected enum STAGE {
		INIT, MOVE, HOVER, TAKEOFF, LAND, GOAL, STOP
	}

	private STAGE next = null;
	protected STAGE stage = STAGE.INIT;
	private STAGE prev = null;
	protected boolean running = false;
	boolean colliding = false;

	private enum OPMODE {
		GO_TO
	}

	private OPMODE mode = OPMODE.GO_TO;

	private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
	private volatile MotionParameters param = DEFAULT_PARAMETERS;
	//need to pass some more parameteres into this param
	//	MotionParameters.Builder settings = new MotionParameters.Builder();


	//	private volatile MotionParameters param = settings.build();

	public MotionAutomaton_quadcopter(GlobalVarHolder gvh, BluetoothInterface bti) {
		super(gvh.id.getName());
		this.gvh = gvh;
		this.bti = bti;

	}

	public void goTo(ItemPosition dest, ObstacleList obsList) {
		goTo(dest);
	}

	public void goTo(ItemPosition dest) {
		if((inMotion && !this.destination.equals(dest)) || !inMotion) {
			done = false;
			this.destination = new ItemPosition(dest.name, dest.getX(), dest.getY(), dest.getZ());
			//this.destination = dest;
			this.mode = OPMODE.GO_TO;
			startMotion();
		}
	}

	@Override
	public synchronized void start() {
		super.start();
		gvh.log.d(TAG, "STARTED!");
	}

	@Override
	public void run() {
		super.run();
		gvh.threadCreated(this);
		// some control parameters
		double kp = 0.00033;
		double kd = 0.0006;
		double kpz = 0.00033;
		double kdz = 0.0006;
		while(true) {
			//			gvh.gps.getObspointPositions().updateObs();
			if(running) {
				mypos = (Model_quadcopter)gvh.plat.getModel();
//				System.out.println(mypos.toString());
				System.out.printf("mypos (%d, %d) \n", mypos.getX(), mypos.getY());
				System.out.printf("destination (%d, %d) \n", destination.getX(), destination.getY());
				int distance = (int) Math.sqrt(Math.pow((mypos.getX() - destination.getX()),2) + Math.pow((mypos.getY() - destination.getY()), 2));
				System.out.println("distance:" + distance);
				//int distance = mypos.distanceTo(destination);
				if(mypos.gaz < -50){
			//		System.out.println("going down");
				}
				colliding = (stage != STAGE.LAND && mypos.gaz < -50);

				if(!colliding && stage != null) {
					switch(stage) {
					case INIT:
						if(mode == OPMODE.GO_TO) {
							if(mypos.getZ() < safeHeight){
								// just a safe distance from ground
								takeOff();
								next = STAGE.TAKEOFF;
							}
							else{
								if(distance <= param.GOAL_RADIUS) {
									System.out.println(">>>Distance: " + distance + " - GOAL_RADIUS " + param.GOAL_RADIUS);
									next = STAGE.GOAL;
								}
								else{
									next = STAGE.MOVE;
								}
							}
						}
						break;
					case MOVE:
						if(mypos.getZ() < safeHeight){
							// just a safe distance from ground
							takeOff();
							next = STAGE.TAKEOFF;
							break;
						}
						if(distance <= param.GOAL_RADIUS) {
							System.out.println(">>>Distance: " + distance + " - GOAL_RADIUS " + param.GOAL_RADIUS);
							next = STAGE.GOAL;
						}
						else{
							double Ax_d, Ay_d = 0.0;
							double Ryaw, Rroll, Rpitch, Rvs, Ryawsp = 0.0;
							//		System.out.println(destination.getX - mypos.getX + " , " + mypos.v_x);
							Vector3f A_d = destination.getPos().subtract(mypos.getPos()).toVector3f().scale(kp)
									.subtract(mypos.getVelocity().scale(kd));
							Ryaw = Math.atan2(destination.getY() - mypos.getY(), destination.getX() - mypos.getX());
							Ryawsp = kpz * ((Ryaw - Math.toRadians(mypos.yaw)));
							Rroll = Math.asin((A_d.getY() * Math.cos(Math.toRadians(mypos.yaw)) - A_d.getX() * Math.sin(Math.toRadians(mypos.yaw))) % 1);
							Rpitch = Math.asin( (-A_d.getY() * Math.sin(Math.toRadians(mypos.yaw)) - A_d.getX() * Math.cos(Math.toRadians(mypos.yaw))) / (Math.cos(Rroll)) % 1);
							Rvs = (kpz * (destination.getZ() - mypos.getZ()) - kdz * mypos.getVelocity().getZ());
						//	System.out.println(Ryaw + " , " + Ryawsp + " , " +  Rroll  + " , " +  Rpitch + " , " + Rvs);

							setControlInputRescale(Math.toDegrees(Ryawsp),Math.toDegrees(Rpitch)%360,Math.toDegrees(Rroll)%360,Rvs);
							//setControlInput(Ryawsp/param.max_yaw_speed, Rpitch%param.max_pitch_roll, Rroll%param.max_pitch_roll, Rvs/param.max_gaz);
							//next = STAGE.INIT;
						}
						break;
					case HOVER:
						setControlInput(0,0,0, 0);
						// do nothing
						break;
					case TAKEOFF:
						switch(mypos.getZ() /(safeHeight/2)){
						case 0:// 0 - 1/2 safeHeight
							setControlInput(0,0,0,1);
							break;
						case 1: // 1/2- 1 safeHeight
							setControlInput(0,0,0, 0.5);
							break;
						default: // above safeHeight:
							hover();
							if(prev != null){
								next = prev;
							}
							else{
								next = STAGE.HOVER;
							}
							break;
						}
						break;
					case LAND:
						switch(mypos.getZ() /(safeHeight/2)){
						case 0:// 0 - 1/2 safeHeight
							setControlInput(0,0,0,0);
							next = STAGE.STOP;
							break;
						case 1: // 1/2- 1 safeHeight
							setControlInput(0,0,0, -0.05);
							break;
						default:   // above safeHeight
							setControlInput(0,0,0,-0.5);
							break;
						}
						break;
					case GOAL:
						System.out.println("Done flag");
						done = true;
						gvh.log.i(TAG, "At goal!");
						gvh.log.i("DoneFlag", "write");
						if(param.STOP_AT_DESTINATION){
							hover();
							next = STAGE.HOVER;
						}
						running = false;
						inMotion = false;
						break;
					case STOP:
						gvh.log.i("FailFlag", "write");
						System.out.println("STOP");
						motion_stop();
						//do nothing
					}
					if(next != null) {
						prev = stage;
						stage = next;
//						System.out.println("Stage transition to " + stage.toString() + ", the previous stage is "+ prev);

						gvh.log.i(TAG, "Stage transition to " + stage.toString());
						gvh.trace.traceEvent(TAG, "Stage transition", stage.toString(), gvh.time());
					}
					next = null;
				}

				if((colliding || stage == null) ) {
					gvh.log.i("FailFlag", "write");
					done = false;
					motion_stop();
				//	land();
				//	stage = STAGE.LAND;
				}
			}
			gvh.sleep(param.AUTOMATON_PERIOD);
		}
	}

	public void cancel() {
		running = false;
		bti.disconnect();
	}

	@Override
	public void motion_stop() {
		//land();
		//stage = STAGE.LAND;
		this.destination = null;
		running = false;
		inMotion = false;
	}

	public void takePicture(){}

	@Override
	public void motion_resume() {
		running = true;
	}

	private void startMotion() {
		running = true;
		stage = STAGE.INIT;
		inMotion = true;
	}

	protected void sendMotionEvent(int motiontype, int... argument) {
		// TODO: This might not be necessary
		gvh.trace.traceEvent(TAG, "Motion", Arrays.toString(argument), gvh.time());
		gvh.sendRobotEvent(Event.MOTION, motiontype);
	}

	private void setControlInputRescale(double yaw_v, double pitch, double roll, double gaz){
		setControlInput(rescale(yaw_v, mypos.max_yaw_speed()), rescale(pitch, mypos.max_pitch_roll()), rescale(roll, mypos.max_pitch_roll()), rescale(gaz, mypos.max_gaz()));
	}

	private double rescale(double value, double max_value){
		if(Math.abs(value) > max_value){
			return (Math.signum(value));
		}
		else{
			return value/max_value;
		}
	}

	protected void setControlInput(double yaw_v, double pitch, double roll, double gaz){
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
		//Bluetooth command to control the drone
	//	gvh.log.i(TAG, "control input as, yaw, pitch, roll, thrust " + yaw_v + ", " + pitch + ", " +roll + ", " +gaz);
		/*
		if(running) {
			if(velocity != 0) {
				sendMotionEvent(Common.MOT_STRAIGHT, velocity);
			} else {
				sendMotionEvent(Common.MOT_STOPPED, 0);
			}
			bti.send(BluetoothCommands.straight(velocity));
		}
		 */
	}

	/**
	 *  	take off from ground
	 */
	protected void takeOff(){
		//Bluetooth command to control the drone
		gvh.log.i(TAG, "Drone taking off");
	}

	/**
	 * land on the ground
	 */
	protected void land(){
		//Bluetooth command to control the drone
		gvh.log.i(TAG, "Drone landing");
	}

	/**
	 * hover at current position
	 */
	protected void hover(){
		//Bluetooth command to control the drone
		gvh.log.i(TAG, "Drone hovering");
	}

	@Override
	public void turnTo(ItemPosition dest) {
		throw new IllegalArgumentException("quadcopter does not have a corresponding turn to");
	}

	@Override
	public void setParameters(MotionParameters param) {
		// TODO Auto-generated method stub
	}


	/**
	 * Slow down linearly upon coming within R_slowfwd of the goal
	 *
	 *
	 * @return
	 */
	/*
	private int LinSpeed(int distance) {
		if(distance > param.SLOWFWD_RADIUS)
			return param.LINSPEED_MAX;
		if(distance > param.GOAL_RADIUS && distance <= param.SLOWFWD_RADIUS) {
			return param.LINSPEED_MIN + (int) ((distance - param.GOAL_RADIUS) * linspeed);
		}
		return param.LINSPEED_MIN;
	}

	// Detects an imminent collision with another robot or with any obstacles

	@Override
	public void setParameters(MotionParameters param) {
		this.param = param;/		this.linspeed = (double) (param.LINSPEED_MAX - param.LINSPEED_MIN) / Math.abs((param.SLOWFWD_RADIUS - param.GOAL_RADIUS));
		this.turnspeed = (param.TURNSPEED_MAX - param.TURNSPEED_MIN) / (param.SLOWTURN_ANGLE - param.SMALLTURN_ANGLE);
	}
	 */

	@Override
	public void userControl(ItemPosition dest, ObstacleList obs){ }

	@Override
	public void receivedKeyInput(String key){
		curKey = key;
	}
}
