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

        while(true) {
            //			gvh.gps.getObspointPositions().updateObs();
            if(running) {
                System.out.printf("drone (%d, %d) \n", drone.getX(), drone.getY());
                System.out.printf("destination (%d, %d) \n", destination.getX(), destination.getY());
                int distance = (int)drone.distanceTo2D(destination);
                System.out.println("distance:" + distance);

                if (drone.getGaz() < -50){
                    // System.out.println("going down");
                }
                boolean colliding = (stage != STAGE.LAND && drone.getGaz() < -50);

                if(!colliding && stage != null) {
                    switch(stage) {
                        case INIT:
                            stageInit(distance);
                            break;
                        case MOVE:
                            stageMove(distance);
                            break;
                        case ROTATOR:
                            stageRotator();
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
                    } else if ((drone.getYaw() >= 100 || drone.getYaw() <= 80) && drone.getZ() < safeHeight && stage != STAGE.ROTATOR) {
                        next = STAGE.ROTATOR;
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

    private void setControlInputRescale(double yaw_v, double pitch, double roll, double gaz){
        setControlInput(rescale(yaw_v, drone.max_yaw_speed()), rescale(pitch, drone.max_pitch_roll()), rescale(roll, drone.max_pitch_roll()), rescale(gaz, drone.max_gaz()));
    }

    protected double calculateYaw() {
        // this method calculates a yaw correction, to keep the drone's yaw angle near 90 degrees
        if(drone.getYaw() > 93) {
            return 5;
        }
        else if(drone.getYaw() < 87) {
            return -5;
        }
        else {
            return 0;
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

    protected abstract void rotateDrone();

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
        // some control parameters
        final double kp = 0.00033;
        final double kd = 0.0006;
        final double kpz = 0.00033;
        final double kdz = 0.0006;

        if (drone.getZ() < safeHeight){
            // just a safe distance from ground
            takeOff();
            next = STAGE.TAKEOFF;
            return;
        }

        if (distance <= param.GOAL_RADIUS) {
            System.out.println(">>>Distance: " + distance + " - GOAL_RADIUS " + param.GOAL_RADIUS);
            next = STAGE.GOAL;
        } else {
            double Ryaw, Rroll, Rpitch, Rvs, Ryawsp = 0.0;
            //		System.out.println(destination.getX - mypos.getX + " , " + mypos.v_x);
            Vector3f A_d = destination.getPos().subtract(drone.getPos()).toVector3f().scale(kp)
                    .subtract(drone.getVelocity().scale(kd));
            Ryaw = atan2(destination.getY() - drone.getY(), destination.getX() - drone.getX());

            final double yawRad = toRadians(drone.getYaw());
            Ryawsp = kpz * (Ryaw - yawRad);
            Rroll = asin((A_d.getY() * cos(yawRad) - A_d.getX() * sin(yawRad)) % 1);
            Rpitch = asin((-A_d.getY() * sin(yawRad) - A_d.getX() * cos(yawRad)) / cos(Rroll) % 1);
            Rvs = (kpz * (destination.getZ() - drone.getZ()) - kdz * drone.getVelocity().getZ());
            //	System.out.println(Ryaw + " , " + Ryawsp + " , " +  Rroll  + " , " +  Rpitch + " , " + Rvs);


            setControlInputRescale(toDegrees(Ryawsp), angleWrap(toDegrees(Rpitch)),
                    angleWrap(toDegrees(Rroll)), Rvs);
            //next = STAGE.INIT;*/

            // TODO convert linear horizontal and vertical thrust to appropriate angular pitch and roll
            double rollCommand = PID_x.getOutput(drone.getX(), destination.getX());
            double pitchCommand = PID_y.getOutput(drone.getY(), destination.getY());
            double yawCommand = calculateYaw();
            double gazCommand = 0;
            gvh.log.d("POSITION DEBUG", "My Position: " + drone.getX() + " " + drone.getY());
            gvh.log.d("POSITION DEBUG", "Destination: " + destination.getX() + " " + destination.getY());

            setControlInputRescale(yawCommand, pitchCommand, rollCommand, gazCommand);
            // TD_NATHAN: check and resolve: was mypos.angle
            // that was the correct solution, has been resolved
        }
    }

    private void stageRotator() {
        if(drone.getYaw() <= 93 && drone.getYaw() >= 87){
            next = STAGE.MOVE;
        } else{
            rotateDrone();
        }
    }

    private void stageHover(double distance) {
        setControlInput(0,0,0, 0);
        // do nothing

        if(distance <= param.GOAL_RADIUS) {
            hover();
        }
        else{
            // TODO convert linear horizontal and vertical thrust to appropriate angular pitch and roll
            double rollCommand = PID_x.getOutput(drone.getX(), destination.getX());
            double pitchCommand = PID_y.getOutput(drone.getY(), destination.getY());
            double yawCommand = calculateYaw();
            double gazCommand = 0;
            setControlInputRescale(yawCommand, pitchCommand, rollCommand, gazCommand);
        }
    }

    private void stageTakeoff() {
        switch(drone.getZ()/(safeHeight/2)){
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
                    System.out.println("hover");
                    next = STAGE.HOVER;
                }
                break;
        }
    }

    private void stageLand() {
        switch(drone.getZ()/(safeHeight/2)){
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
            drone.setPitch(drone.getPitch() + .01);
        }
        System.out.println(drone.getYaw());
        setControlInput(drone.getYaw(), drone.getPitch(), drone.getRoll(), drone.getGaz());
    }
}
