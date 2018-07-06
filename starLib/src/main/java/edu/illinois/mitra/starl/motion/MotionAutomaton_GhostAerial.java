package edu.illinois.mitra.starl.motion;

import java.util.*;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
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
public class MotionAutomaton_GhostAerial extends RobotMotion {
    protected static final String TAG = "MotionAutomaton";
    protected static final String ERR = "Critical Error";
    final int safeHeight = 500;
    private boolean landed = true;

    protected GlobalVarHolder gvh;
    //protected GhostAerialBTI bti;
    protected DroneBTI bti;

    // Motion tracking
    protected ItemPosition destination;
    private Model_GhostAerial mypos;

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
    double Kpx = 0.0314669809792096;
    double Kpy = 0.0314669809792096;
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
        GO_TO, USER_CONTROL
    }

    private OPMODE mode = OPMODE.GO_TO;

    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    //need to pass some more parameteres into this param
    //	MotionParameters.Builder settings = new MotionParameters.Builder();


    //	private volatile MotionParameters param = settings.build();

    public MotionAutomaton_GhostAerial(GlobalVarHolder gvh, GhostAerialBTI bti) {
        super(gvh.id.getName());
        this.gvh = gvh;
        this.bti = bti;

    }

    public void goTo(ItemPosition dest, ObstacleList obsList) {
        goTo(dest);
    }

    public void goTo(ItemPosition dest) {
        if(!inMotion || !this.destination.equals(dest)) {
            done = false;
            this.destination = new ItemPosition(dest.name, dest.getX(), dest.getY(), dest.getZ());
            gvh.log.d(TAG, "Going to X: " + Integer.toString(dest.getX()) + ", Y: " + Integer.toString(dest.getY()));
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
                mypos = (Model_GhostAerial)gvh.gps.getMyPosition();
                System.out.printf("mypos (%d, %d) \n", mypos.getX(), mypos.getY());
                System.out.printf("destination (%d, %d) \n", destination.getX(), destination.getY());
                //int distance = (int) Math.sqrt(Math.pow((mypos.getX - destination.getX),2) + Math.pow((mypos.getY - destination.getY), 2));
                int distance = (int)mypos.distanceTo2D(destination);
                System.out.println("distance:" + distance);

                colliding = (stage != STAGE.LAND && mypos.gaz < -50);

                if(!colliding && stage != null) {
                    if(stage != prev){
                        gvh.log.e(TAG, "Stage is: " + stage.toString());
                    }
                    switch(stage) {
                        case INIT:
                            if(mode == OPMODE.GO_TO) {
                                PID_x.reset();
                                PID_y.reset();
                                //setMaxTilt(5);
                                if(mypos.getZ() < safeHeight || landed){
                                    // just a safe distance from ground
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
                                /*double Ax_d, Ay_d = 0.0;
                                double Ryaw, Rroll, Rpitch, Rvs, Ryawsp = 0.0;
                                //		System.out.println(destination.getX - mypos.getX + " , " + mypos.v_x);
                                Ax_d = (kpx * (destination.getX - mypos.getX) - kdx * mypos.v_x) ;
                                Ay_d = (kpy * (destination.getY - mypos.getY) - kdy * mypos.v_y) ;
                                Ryaw = Math.atan2(destination.getY - mypos.getY, destination.getX - mypos.getX);
                                //Ryaw = Math.atan2((destination.getY - mypos.getX), (destination.getX - mypos.getY));
                                Ryawsp = kpz * ((Ryaw - Math.toRadians(mypos.yaw)));
                                Rroll = Math.asin((Ay_d * Math.cos(Math.toRadians(mypos.yaw)) - Ax_d * Math.sin(Math.toRadians(mypos.yaw))) %1);
                                Rpitch = Math.asin( (-Ay_d * Math.sin(Math.toRadians(mypos.yaw)) - Ax_d * Math.cos(Math.toRadians(mypos.yaw))) / (Math.cos(Rroll)) %1);
                                Rvs = (kpz * (destination.getZ - mypos.getZ) - kdz * mypos.v_z);
                                //	System.out.println(Ryaw + " , " + Ryawsp + " , " +  Rroll  + " , " +  Rpitch + " , " + Rvs);

                                setControlInputRescaleD(Math.toDegrees(Ryawsp),Math.toDegrees(Rpitch)%360,Math.toDegrees(Rroll)%360,Rvs);
                                //setControlInput(Ryawsp/param.max_yaw_speed, Rpitch%param.max_pitch_roll, Rroll%param.max_pitch_roll, Rvs/param.max_gaz);
                                //next = STAGE.INIT;
                                System.out.printf("WORKING COMMANDS: %f %f %f %f \n",
                                        rescale(Math.toDegrees(Ryawsp), mypos.max_yaw_speed),
                                        rescale(Math.toDegrees(Rpitch)%360, mypos.max_pitch_roll),
                                        rescale(Math.toDegrees(Rroll)%360, mypos.max_pitch_roll),
                                        rescale(Rvs, mypos.max_gaz));*/
                                double rollCommand = PID_x.getCommand(mypos.getX(), destination.getX())*-1;
                                double pitchCommand = PID_y.getCommand(mypos.getY(), destination.getY())*-1;
                                double yawCommand = calculateYaw();
                                double gazCommand = 0;
                                setControlInputRescale(yawCommand, pitchCommand, rollCommand, gazCommand);
                                double maxVal = 50;
                                System.out.printf("Input commands (-1 to 1): %f %f %f %f \n",
                                        rescale(yawCommand, maxVal),
                                        rescale(pitchCommand, maxVal),
                                        rescale(rollCommand, maxVal),
                                        rescale(gazCommand, maxVal));
                            }
                            break;
                        case HOVER:
                            if(distance <= param.GOAL_RADIUS) {
                                hover();
                                double yawCommand = calculateYaw();
                                setControlInput(yawCommand, 0, 0, 0);
                            }
                            else {
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
                            land();
                            next = STAGE.STOP;
                            break;
                        case GOAL:
                            System.out.println("Done flag");
                            done = true;
                            gvh.log.i(TAG, "At goal!");
                            gvh.log.i("DoneFlag", "write");
                            if(param.STOP_AT_DESTINATION){
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
        land();
        //bti.disconnect();
    }

    @Override
    public void motion_stop() {
        stage = STAGE.HOVER;
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
        setControlInput(rescale(yaw_v, 50), rescale(pitch, 50), rescale(roll, 50), rescale(gaz, 50));
    }
    private void setControlInputRescaleD(double yaw_v, double pitch, double roll, double gaz){
       // setControlInput(rescale(yaw_v, mypos.max_yaw_speed), rescale(pitch, mypos.max_pitch_roll), rescale(roll, mypos.max_pitch_roll), rescale(gaz, mypos.max_gaz));
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
        bti.setControlInput(yaw_v, pitch, roll, gaz);
        gvh.log.i(TAG, "control input as, yaw, pitch, roll, thrust " + yaw_v + ", " + pitch + ", " +roll + ", " +gaz);
    }

    /**
     *  	take off from ground
     */
    protected void takeOff(){
        bti.sendTakeoff();
        gvh.log.i(TAG, "Drone taking off");
    }

    /**
     * land on the ground
     */
    protected void land(){
        bti.sendLanding();
        gvh.log.i(TAG, "Drone landing");
    }

    /**
     * hover at current position
     */
    protected void hover(){
        bti.hover();
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
     * atparam distance
     * atreturn
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

    @Override
    public void userControl(ItemPosition dest, ObstacleList obs){ }

    @Override
    public void receivedKeyInput(String key){
        curKey = key;
    }
}


