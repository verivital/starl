package edu.illinois.mitra.starl.motion;

//import android.util.Log;

import java.util.Arrays;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model_Mavic;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PIDController;
import edu.illinois.mitra.starl.objects.Vector3f;

//import edu.illinois.mitra.starl.models.Model_quadcopter;

/**
 * Created by VerivitalLab on 2/19/2016.
 */
public class MotionAutomaton_Mavic extends RobotMotion {
    protected static final String TAG = "MotionAutomaton";
    protected static final String ERR = "Critical Error";
    final int safeHeight = 150;
    private boolean landed = true;

    protected GlobalVarHolder gvh;
    protected DroneBTI bti;

    // Motion tracking
    protected ItemPosition destination;
    private Model_Mavic mypos; // TODO complete model for mavic

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

    private OPMODE mode = OPMODE.GO_TO;

    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    //need to pass some more parameteres into this param
    //	MotionParameters.Builder settings = new MotionParameters.Builder();


    //	private volatile MotionParameters param = settings.build();

    public MotionAutomaton_Mavic(GlobalVarHolder gvh, DjiController bti) {
        super(gvh.id.getName());
        this.gvh = gvh;
        this.bti = bti;
    }

    public void goTo(ItemPosition dest, ObstacleList obsList) {
        goTo(dest);
    }

    public void goTo(ItemPosition dest) {
        if((inMotion && !this.destination.equals(dest)) || !inMotion) {
            this.destination = new ItemPosition(dest.name, dest.getX(), dest.getY(), dest.getZ());
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

/*
    @Override
    public void run() {
        super.run();
        gvh.threadCreated(this);
        while(true) {
            //			gvh.gps.getObspointPositions().updateObs();
            if(running) {
                mypos = (Model_Mavic)gvh.gps.getMyPosition(); // TD_NATHAN: check. I fixed it.
   //             if(mypos == null) { continue;}
                // if you change to 3D waypoints, use distanceTo instead of distanceTo2D
                int distance = mypos.distanceTo2D(destination);
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
                                double rollCommand = PID_x.getCommand(mypos.getX, destination.getX);
                                double pitchCommand = PID_y.getCommand(mypos.getY, destination.getY);
                                double yawCommand = calculateYaw();
                                System.out.println(rollCommand);
                                System.out.println(pitchCommand);
                                System.out.println(yawCommand);
                                System.out.println();
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
                                double rollCommand = PID_x.getCommand(mypos.getX, destination.getX);
                                double pitchCommand = PID_y.getCommand(mypos.getY, destination.getY);
                                double yawCommand = calculateYaw();
                                double gazCommand = 0;
                                setControlInput(yawCommand, pitchCommand, rollCommand, gazCommand);
                            }
                            break;
                        case TAKEOFF:
                            takeOff();
                            //takePicture();
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
*/

    @Override
    public void run() {
        super.run();
        gvh.threadCreated(this);
        // some control parameters
        double kp, kpz, kd, kdz;
        kp = kpz = 0.00033;
        kd = kdz = 0.0006;
        while(true) {
            //			gvh.gps.getObspointPositions().updateObs();
            if(running) {
                mypos = (Model_Mavic) gvh.plat.getModel();
//				System.out.println(mypos.toString());
                int distance = (int) Math.sqrt(Math.pow((mypos.getX() - destination.getX()),2) + Math.pow((mypos.getY() - destination.getY()), 2));
                //int distance = mypos.distanceTo(destination);
                if(mypos.gaz < -50){
                    //		System.out.println("going down");
                }
                colliding = (stage != MotionAutomaton_Mavic.STAGE.LAND && mypos.gaz < -50);

                if(!colliding && stage != null) {
                    switch(stage) {
                        case INIT:
                            if(mode == MotionAutomaton_Mavic.OPMODE.GO_TO) {
                                if(mypos.getZ() < safeHeight){
                                    // just a safe distance from ground
                                    takeOff();
                                    next = MotionAutomaton_Mavic.STAGE.TAKEOFF;
                                }
                                else{
                                    if(distance <= param.GOAL_RADIUS) {
                                        next = MotionAutomaton_Mavic.STAGE.GOAL;
                                    }
                                    else{
                                        next = MotionAutomaton_Mavic.STAGE.MOVE;
                                    }
                                }
                            }
                            break;
                        case MOVE:
                            if(mypos.getZ() < safeHeight){
                                // just a safe distance from ground
                                takeOff();
                                next = MotionAutomaton_Mavic.STAGE.TAKEOFF;
                                break;
                            }
                            if(distance <= param.GOAL_RADIUS) {
                                next = MotionAutomaton_Mavic.STAGE.GOAL;
                            }
                            else{
                                double Ryaw, Rroll, Rpitch, Rvs, Ryawsp = 0.0;
                                Vector3f A_d = destination.getPos().subtract(mypos.getPos()).toVector3f().scale(kp)
                                        .subtract(mypos.getVelocity().scale(kd));
                                Ryaw = Math.atan2(destination.getY() - mypos.getY(), destination.getX() - mypos.getX());
                                //Ryaw = Math.atan2((destination.getY - mypos.getX), (destination.getX - mypos.getY));
                                Ryawsp = kpz * ((Ryaw - Math.toRadians(mypos.yaw)));
                                Rroll = Math.asin((A_d.getY() * Math.cos(Math.toRadians(mypos.yaw)) - A_d.getX() * Math.sin(Math.toRadians(mypos.yaw))) %1);
                                Rpitch = Math.asin( (-A_d.getY() * Math.sin(Math.toRadians(mypos.yaw)) - A_d.getX() * Math.cos(Math.toRadians(mypos.yaw))) / (Math.cos(Rroll)) %1);
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
                                        next = MotionAutomaton_Mavic.STAGE.HOVER;
                                    }
                                    break;
                            }
                            break;
                        case LAND:
                            switch(mypos.getZ() /(safeHeight/2)){
                                case 0:// 0 - 1/2 safeHeight
                                    setControlInput(0,0,0,0);
                                    next = MotionAutomaton_Mavic.STAGE.STOP;
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
                            done = true;
                            gvh.log.i(TAG, "At goal!");
                            gvh.log.i("DoneFlag", "write");
                            if(param.STOP_AT_DESTINATION){
                                hover();
                                next = MotionAutomaton_Mavic.STAGE.HOVER;
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
        bti.setControlInput(yaw_v, pitch, roll, gaz);
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

    public void takePicture(){
        //Bluetooth command to control the drone
        gvh.log.i(TAG, "Taking picture");
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
            return 1;
        }
        else if(mypos.yaw < 87) {
            return -1;
        }
        else {
            return 0;
        }
    }

    private void setMaxTilt(float val) {
        //bti.setMaxTilt(val);
        //TODO figure out a way to do this w mavic
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
