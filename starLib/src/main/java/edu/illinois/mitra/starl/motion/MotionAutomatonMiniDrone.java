package edu.illinois.mitra.starl.motion;

//import android.util.Log;

import java.util.Arrays;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;

//import edu.illinois.mitra.starl.models.Model_quadcopter;

/**
 * Created by VerivitalLab on 2/19/2016.
 */
public class MotionAutomatonMiniDrone extends RobotMotion {
    protected static final String TAG = "MotionAutomaton";
    protected static final String ERR = "Critical Error";
    final int safeHeight = 150;
    private boolean landed = true;

    protected GlobalVarHolder gvh;
    protected MiniDroneBTI bti;

    // Motion tracking
    protected ItemPosition destination;
    private Model_quadcopter mypos; // TD_NATHAN: probably need to create a minidrone one of these objects, as I think this is for AR drone...?

    //PID controller parameters
    double saturationLimit = 50;
    double windUpLimit = 185;
    int filterLength = 8;
    /*double Kpx = 0.2;
    double Kpy = 0.2;
    double Kix = 0.04;
    double Kiy = 0.04;
    double Kdx = 0.4;
    double Kdy = 0.45;*/
    // the ones below work pretty well
    double Kpx = 0.0714669809792096;
    double Kpy = 0.0714669809792096;
    double Kix = 0.0110786899216426;
    double Kiy = 0.0110786899216426;
    double Kdx = 0.113205037832174;
    double Kdy = 0.113205037832174;

    PIDController PID_x = new PIDController(Kpx, Kix, Kdx, saturationLimit, windUpLimit, filterLength);
    PIDController PID_y = new PIDController(Kpy, Kiy, Kdy, saturationLimit, windUpLimit, filterLength);


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

    public void takePicture(){}

    private OPMODE mode = OPMODE.GO_TO;

    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    //need to pass some more parameteres into this param
    //	MotionParameters.Builder settings = new MotionParameters.Builder();


    //	private volatile MotionParameters param = settings.build();

    public MotionAutomatonMiniDrone(GlobalVarHolder gvh, MiniDroneBTI bti) {
        super(gvh.id.getName());
        this.gvh = gvh;
        this.bti = bti;
    }

    public void goTo(ItemPosition dest, ObstacleList obsList) {
        goTo(dest);
    }

    public void goTo(ItemPosition dest) {
        if((inMotion && !this.destination.equals(dest)) || !inMotion) {
            this.destination = new ItemPosition(dest.name, dest.getX(), dest.getY(),0);
            gvh.log.d(TAG, "Going to X: " + Integer.toString(dest.getX()) + ", Y: " + Integer.toString(dest.getY()));
      //      Log.d(TAG, "Going to X: " + Integer.toString(dest.getX) + ", Y: " + Integer.toString(dest.getY));
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
        while(true) {
            //			gvh.gps.getObspointPositions().updateObs();
            if(running) {
                mypos = (Model_quadcopter)gvh.gps.getMyPosition(); // TD_NATHAN: check. I fixed it.
   //             if(mypos == null) { continue;}
                // if you change to 3D waypoints, use distanceTo instead of distanceTo2D
                int distance = (int)mypos.distanceTo2D(destination);
                colliding = false;

                if(!colliding && stage != null) {
                    if(stage != prev)
                        gvh.log.e(TAG, "Stage is: " + stage.toString());

                    switch(stage) {
                        case INIT:
                            if(mode == OPMODE.GO_TO) {
                                PID_x.reset();
                                PID_y.reset();
                                setMaxTilt(5); // TODO: add max tilt to motion paramters class
                                if(landed){
                                    next = STAGE.TAKEOFF;
                                }
                                else{
                                    if(distance <= param.GOAL_RADIUS) {
                                        next = STAGE.GOAL;
                                    }
                                    else{
                                        next = STAGE.MOVE;
                                    }
                                }
                            }
                            break;
                        case MOVE:
                            if(distance <= param.GOAL_RADIUS) {
                                next = STAGE.GOAL;
                            }
                            else{
                                double rollCommand = PID_x.getCommand(mypos.getX(), destination.getX());
                                double pitchCommand = PID_y.getCommand(mypos.getY(), destination.getY());
                                double yawCommand = calculateYaw();
                                double gazCommand = 0;
                                setControlInput(yawCommand, pitchCommand, rollCommand, gazCommand);
                                // TD_NATHAN: check and resolve: was mypos.angle
                                // that was the correct solution, has been resolved
                            }
                            break;
                        case HOVER:
                            if(distance <= param.GOAL_RADIUS) {
                                hover();
                                double yawCommand = calculateYaw();
                                setControlInput(yawCommand, 0, 0, 0);
                            }
                            else{
                                double rollCommand = PID_x.getCommand(mypos.getX(), destination.getX());
                                double pitchCommand = PID_y.getCommand(mypos.getY(), destination.getY());
                                double yawCommand = calculateYaw();
                                double gazCommand = 0;
                                setControlInput(yawCommand, pitchCommand, rollCommand, gazCommand);
                            }
                            break;
                        case TAKEOFF:
                            takeOff();
                            landed = false;
                            next = STAGE.MOVE;
                            break;
                        case LAND:
                            land();
                            break;
                        case GOAL:
                            gvh.log.i(TAG, "At goal!");
                            if(param.STOP_AT_DESTINATION){
                                next = STAGE.HOVER;
                            }
                           // running = false;
                            inMotion = false;
                            break;
                        case STOP:
                            //do nothing
                    }
                    if(next != null) {
                        prev = stage;
                        stage = next;
                        System.out.println("Stage transition to " + stage.toString() + "previous stage is "+ prev);

                        gvh.log.i(TAG, "Stage transition to " + stage.toString());
                        gvh.trace.traceEvent(TAG, "Stage transition", stage.toString(), gvh.time());
                    }
                    next = null;
                }

                if((colliding || stage == null) ) {
                    land();
                    stage = STAGE.LAND;
                }
            }
            gvh.sleep(param.AUTOMATON_PERIOD);
        }
    }

    public void cancel() {
        running = false;
        land();
    }

    @Override
    public void motion_stop() {
        stage = STAGE.HOVER;
        this.destination = null;
        running = false;
        inMotion = false;
    }

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

    protected void setControlInput(double yaw_v, double pitch, double roll, double gaz){
        //Bluetooth command to control the drone
        bti.setRoll((byte) roll);
        bti.setPitch((byte) pitch);
        bti.setYaw((byte) yaw_v);
        // currently not moving to 3-D waypoints, so not sending a gaz command
        // if in the future you want to send one, uncomment the following line
        bti.setThrottle((byte) gaz);
        gvh.log.i(TAG, "control input as, yaw, pitch, roll, thrust " + yaw_v + ", " + pitch + ", " +roll + ", " +gaz);
    }

    /**
     *  	take off from ground
     */
    protected void takeOff(){
        //Bluetooth command to control the drone
        bti.sendTakeoff();
        gvh.log.i(TAG, "Drone taking off");
    }

    /**
     * land on the ground
     */
    protected void land(){
        //Bluetooth command to control the drone
        bti.sendLanding();
        gvh.log.i(TAG, "Drone landing");
    }

    /**
     * hover at current position
     */
    protected void hover(){
        //Bluetooth command to control the drone
        bti.hover();
        gvh.log.i(TAG, "Drone hovering");
    }

    private double calculateYaw() {
        // this method calculates a yaw correction, to keep the drone's yaw angle near 90 degrees
        if(mypos.yaw > 93) {
            return 5;
        }
        else if(mypos.yaw < 87) {
            return -5;
        }
        else {
            return 0;
        }
    }

    private void setMaxTilt(float val) {
        bti.setMaxTilt(val);
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
     * @param distance
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

}
