package edu.illinois.mitra.starl.motion;

import java.util.*;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.objects.*;
import static edu.illinois.mitra.starl.objects.Common.angleWrap;
import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.atan2;
import static java.lang.Math.asin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

/**
 * TODO: Remove unncessary methods/cleanup, fix PID Controller.
 * Base logic for all drone motion, allowing apps to use a goTo method to reach a specific point, or control motion with arrow keys.
 * Extended by RealisticSimMotionAutomaton_Drone for simulation and RealisticMotionAutomaton_Drone for real life applications.
 */
public abstract class MotionAutomaton_Drone extends RobotMotion {
    protected static final String TAG = "MotionAutomaton";
    protected static final String ERR = "Critical Error";
    private static final int safeHeight = 500;
    private boolean abort = false;

    // control input is not sent right away, but delayed until the end of the loop
    private double storedYaw, storedPitch, storedRoll, storedGaz;
    boolean storeChanged;
    protected void storeControlInput(double yaw, double pitch, double roll, double gaz) {
        storedYaw = yaw;
        storedPitch = pitch;
        storedRoll = roll;
        storedGaz = gaz;
        storeChanged = true;
    }
    protected void storeYaw(double yaw) {
        storedYaw = yaw;
        storeChanged = true;
    }
    protected void storePitch(double pitch) {
        storedPitch = pitch;
        storeChanged = true;
    }
    protected void storeRoll(double roll) {
        storedRoll = roll;
        storeChanged = true;
    }
    protected void storeGaz(double gaz) {
        storedGaz = gaz;
        storeChanged = true;
    }

    protected GlobalVarHolder gvh;

    // Motion tracking
    protected ItemPosition destination;
    protected final Model_Drone drone;

    protected enum STAGE {
        INIT, MOVE, ROTATOR, HOVER, TAKEOFF, LAND, GOAL, STOP, USER_CONTROL
    }

    private STAGE next = null;
    protected volatile STAGE stage = STAGE.INIT;
    private STAGE prev = null;
    protected volatile boolean running = false;

    private final PIDController PID_x;
    private final PIDController PID_y;

    private enum OPMODE {
        GO_TO, USER_CONTROL
    }
    private OPMODE mode = OPMODE.GO_TO;

    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    //need to pass some more parameteres into this param
    //	MotionParameters.Builder settings = new MotionParameters.Builder();


    public MotionAutomaton_Drone(GlobalVarHolder gvh) {
        super(gvh.id.getName());
        this.gvh = gvh;
        this.drone = (Model_Drone)gvh.plat.model;

        PIDParams pidParams = drone.getPIDParams();
        PID_x = new PIDController(pidParams);
        PID_y = new PIDController(pidParams);
    }

    public void goTo(ItemPosition dest, ObstacleList obsList) {
        goTo(dest);
    }

    public void goTo(ItemPosition dest) {
        if(!inMotion || !this.destination.equals(dest)) {
            done = false;
            this.destination = dest;
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
        gvh.threadCreated(this);
        // cannot call in constructor
        setMaxTilt((float)drone.max_pitch_roll());

        while(true) {
            //			gvh.gps.getObspointPositions().updateObs();
            if(running) {
                //System.out.printf("drone (%d, %d) \n", drone.getX(), drone.getY());
                //System.out.printf("destination (%d, %d) \n", destination.getX(), destination.getY());
                int distance = (int)drone.distanceTo2D(destination);
                //System.out.println("distance:" + distance);

                boolean colliding = (stage != STAGE.LAND && drone.getGaz() < -50);

                if(!colliding && stage != null) {
                    switch(stage) {
                        case INIT:
                            stageInit(distance);
                            break;
                        case MOVE:
                            stageMove(distance);
                            break;
                        case HOVER:
                            stageHover(distance);
                            break;
                        case TAKEOFF:
                            stageTakeoff();
                            break;
                        case LAND:
                            stageLand();
                            break;
                        case GOAL:
                            stageGoal();
                            break;
                        case STOP:
                            stageStop();
                            break;
                        case USER_CONTROL:
                            stageUserControl();
                            break;
                    }

                    if(abort) {
                        next = STAGE.LAND;
                    }
                    if(next != null) {
                        prev = stage;
                        stage = next;
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
            if (storeChanged) {
                // Call overridden implementation of setControlInput with stored values
                setControlInput(storedYaw, storedPitch, storedRoll, storedGaz);
                storeChanged = false;
            }
            gvh.sleep(param.AUTOMATON_PERIOD);
        }
    }

    public void cancel() {
        running = false;
        land();
    }

    private void startMotion() {
        running = true;
        stage = STAGE.INIT;
        inMotion = true;
    }

    @Override
    public void motion_stop() {
        abort = true;
        inMotion = false;
        this.destination = null;
        running = false;
    }

    @Override
    public void motion_resume() {
        running = true;
    }

    protected void sendMotionEvent(int motiontype, int... argument) {
        // TODO: This might not be necessary
        gvh.trace.traceEvent(TAG, "Motion", Arrays.toString(argument), gvh.time());
        gvh.sendRobotEvent(Event.MOTION, motiontype);
    }

    //private void setControlInputRescale(double yaw_v, double pitch, double roll, double gaz){
    //    setControlInput(rescale(yaw_v, drone.max_yaw_speed()), rescale(pitch, drone.max_pitch_roll()), rescale(roll, drone.max_pitch_roll()), rescale(gaz, drone.max_gaz()));
    //}

    @Override
    public void turnTo(ItemPosition dest) {
        throw new IllegalArgumentException("quadcopter does not have a corresponding turn to");
    }

    @Override
    public void setParameters(MotionParameters param) {
        // TODO Auto-generated method stub
    }

    /**
     * Enables user control when called from App.
     * @param dest -- Location of waypoint
     * @param obs -- Location of obstacles
     */
    @Override
    public void userControl(ItemPosition dest, ObstacleList obs){
        done = false;
        running = true;
        this.destination = new ItemPosition(dest);
        this.mode = OPMODE.USER_CONTROL;
        startMotion();
    }

    /**
     * Receves string representing which key was pressed. "forward" for up arrow, "back" for down arrow,
     * "left" and "right" for arrows, "up" for W, "down" for S, "turnL" for A, "turnR" for D. Once a key is released,
     * String key returns "stop".
     * @param key -- String representing which key was pressed. Changed to "stop" when released.
     */
    @Override
    public void receivedKeyInput(String key){
        curKey = key;
    }

    public void takePicture(){}

    protected final void abort() {
        abort = true;
    }

    protected abstract void setControlInput(double yaw_v, double pitch, double roll, double gaz);

    /**
     *  	take off from ground
     */
    protected abstract void takeOff();

    /**
     * land on the ground
     */
    protected abstract void land();

    /**
     * hover at current position
     */
    protected abstract void hover();

    protected abstract void setMaxTilt(float val);

    protected static double rescale(double value, double max_value){
        if(Math.abs(value) > max_value){
            return (Math.signum(value));
        }
        else{
            return value/max_value;
        }
    }

    /**            _____                   _____
     * x-thrust   |     | drone-x-thrust  |     | pitch
     * ---------->|     |---------------->|     |------->
     * y-thrust   |     | drone-y-thrust  |     | roll
     * ---------->|     |---------------->|_____|------->
     *  yaw       |     |
     *  --------->|_____|
     */
    private void setXYThrust(double thrustX, double thrustY) {
        // Step 1: convert from the world coordinate system to the drone coordinate system
        final double yaw = toRadians(drone.getYaw() - 90);
        // we are assuming for now that the drone coordinate system matches the world coordinate
        // system when drone.getYaw() == 0, and yaw increases counterclockwise
        double droneThrustX = thrustX * cos(yaw) + thrustY * sin(yaw);
        double droneThrustY = thrustX * -sin(yaw) + thrustY * cos(yaw);

        // Step 2: convert from cartesian X and Y thrust values to pitch and roll
        final double limit = sin(toRadians(drone.max_pitch_roll())); // maximum allowed thrust magnitude, < 1
        final double magnitude = Math.hypot(droneThrustX, droneThrustY);
        if (magnitude > limit) {
            // reduce total thrust magnitude to limit
            droneThrustX *= limit / magnitude;
            droneThrustY *= limit / magnitude;
        }
        // Scaled to [-1, 1]
        double rollCommand = asin(droneThrustX) * 2 / Math.PI;
        double pitchCommand = asin(droneThrustY) * 2 / Math.PI;

        storePitch(pitchCommand);
        storeRoll(rollCommand);
    }

    private void stageInit(double distance) {
        PID_x.reset();
        PID_y.reset();
//                            setMaxTilt(2.5f); // TODO: add max tilt to motion parameters class

        if(drone.getZ() < safeHeight){
            // just a safe distance from ground
            takeOff();
            next = STAGE.TAKEOFF;
        }
        else{
            if(distance <= param.GOAL_RADIUS) {
                System.out.println(">>>Distance: " + distance + " - GOAL_RADIUS " + param.GOAL_RADIUS);
                next = STAGE.GOAL;
            }
            else if(mode == OPMODE.GO_TO) {
                next = STAGE.MOVE;
            } else if(mode == OPMODE.USER_CONTROL){
                next = STAGE.USER_CONTROL;
            }
        }
    }

    private void stageMove(double distance) {
        if (drone.getZ() < safeHeight){
            // just a safe distance from ground
            takeOff();
            next = STAGE.TAKEOFF;
            return;
        }

        //if (distance <= param.GOAL_RADIUS) {
        //    System.out.println(">>>Distance: " + distance + " - GOAL_RADIUS " + param.GOAL_RADIUS);
        //    next = STAGE.GOAL;
        //} else {
            double thrustX = PID_x.getOutput(drone.getX(), destination.getX());
            double thrustY = PID_y.getOutput(drone.getY(), destination.getY());
            setXYThrust(thrustX, thrustY);

            //gvh.log.d("POSITION DEBUG", "My Position: " + drone.getX() + " " + drone.getY());
            //gvh.log.d("POSITION DEBUG", "Destination: " + destination.getX() + " " + destination.getY());
        //}
    }

    private void stageHover(double distance) {
        // do nothing

        if(distance <= param.GOAL_RADIUS) {
            hover();
        } else{
            double thrustX = PID_x.getOutput(drone.getX(), destination.getX());
            double thrustY = PID_y.getOutput(drone.getY(), destination.getY());
            setXYThrust(thrustX, thrustY);
        }
    }

    private void stageTakeoff() {
        switch(drone.getZ()/(safeHeight/2)){
            case 0:// 0 - 1/2 safeHeight
                storeGaz(1);
                break;
            case 1: // 1/2- 1 safeHeight
                storeGaz(0.5);
                break;
            default: // above safeHeight:
                hover();
                if(prev != null){
                    next = prev;
                }
                else{
                    System.out.println("hover");
                    next = STAGE.HOVER;
                }
                break;
        }
    }

    private void stageLand() {
        switch(drone.getZ()/(safeHeight/2)){
            case 0:// 0 - 1/2 safeHeight
                storeGaz(0);
                next = STAGE.STOP;
                break;
            case 1: // 1/2- 1 safeHeight
                storeGaz(-0.05);
                break;
            default:   // above safeHeight
                storeGaz(-0.5);
                break;
        }
    }

    private void stageGoal() {
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
    }

    private void stageStop() {
        gvh.log.i("FailFlag", "write");
        System.out.println("STOP");
        motion_stop();
        //do nothing
    }

    private void stageUserControl() {
        if(curKey.equals("forward") && drone.getPitch() <= .9){
            storePitch(storedPitch + .01);
        }
    }
}
