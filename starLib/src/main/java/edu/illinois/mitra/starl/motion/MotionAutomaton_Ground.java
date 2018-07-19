package edu.illinois.mitra.starl.motion;

import java.util.Arrays;
import java.util.Scanner;
import java.util.Vector;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;

public abstract class MotionAutomaton_Ground extends RobotMotion {
    protected static final String TAG = "MotionAutomaton";
    protected static final String ERR = "Critical Error";

    protected final GlobalVarHolder gvh;

    // Motion tracking
    protected ItemPosition destination;
    protected STAGE stage = STAGE.INIT;
    protected boolean running = false;
    private boolean colliding = false;
    private Model_Ground bot;
    private ItemPosition blocker;
    private ObstacleList obsList;
    private STAGE next = null;
    private STAGE prev = null;
    private OPMODE mode = OPMODE.GO_TO;
    private volatile MotionParameters param = MotionParameters.defaultParameters();
    private COLSTAGE colprev = null;
    private COLSTAGE colstage = COLSTAGE.TURN;
    private COLSTAGE colnext = null;
    private COLSTAGE0 colstage0 = COLSTAGE0.STRAIGHT;
    private COLSTAGE0 colnext0 = null;
    private COLSTAGE1 colstage1 = COLSTAGE1.STRAIGHT;
    private COLSTAGE1 colnext1 = null;
    private COLSTAGE2 colstage2 = COLSTAGE2.BACK;
    private COLSTAGE2 colnext2 = null;
    private int col_straightime = 0;
    private int col_backtime = 0;
    private int col_turntime = 0;
    private int RanAngle = 0;
    private double linspeed;
    private double turnspeed;

    public MotionAutomaton_Ground(GlobalVarHolder gvh) {
        super(gvh.id.getName());
        this.gvh = gvh;
        calcSpeeds(); // initialize linspeed and turnspeed
    }

    public void goTo(ItemPosition dest, ObstacleList obsList) {
        if((!inMotion || !this.destination.equals(dest))) {
            this.destination = new ItemPosition(dest.name, dest.getX(), dest.getY(),0);
//            Log.d(TAG, "Going to X: " + Integer.toString(dest.getX) + " Y: " + Integer.toString(dest.getY));
            //this.destination = dest;
            this.mode = OPMODE.GO_TO;
            this.obsList = obsList;
            startMotion();
        }
    }

    public void goTo(ItemPosition dest) {
        Scanner in = new Scanner(gvh.gps.getMyPosition().getName()).useDelimiter("[^0-9]+");
        int index = in.nextInt();
        Vector<ObstacleList> temp = gvh.gps.getViews();
        ObstacleList obsList;
        if(!temp.isEmpty()) {
            obsList = temp.elementAt(index);
        }
        else {
            obsList = new ObstacleList();
        }
        //obsList = new ObstacleList();
        // work in progress here
        goTo(dest, obsList);
    }

    public void turnTo(ItemPosition dest) {
        if(!inMotion || !this.destination.equals(dest)) {
            this.destination = dest;
            this.mode = OPMODE.TURN_TO;
            startMotion();
        }
    }

    @Override
    public void motion_resume() {
        running = true;
    }

    @Override
    public abstract void motion_stop();

    @Override
    public void setParameters(MotionParameters param) {
        this.param = param;
        calcSpeeds();
    }

    @Override
    public void userControl(ItemPosition dest, ObstacleList obs){ }

    @Override
    public synchronized void start() {
        super.start();
        gvh.log.d(TAG, "STARTED!");
    }

    @Override
    public final void run() {
        gvh.threadCreated(this);


        while(true) {
//			gvh.gps.getObspointPositions().updateObs();
            if(running) {
                // why is getModel being used? Think it should be get position.
                //bot = (Model_iRobot)gvh.plat.getModel();
                bot = (Model_Ground)gvh.gps.getMyPosition();
                int distance = (int) bot.distanceTo(destination);
                int angle = bot.angleTo(destination);
                int absangle = Math.abs(angle);
                switch(param.COLAVOID_MODE) {
                    case BUMPERCARS:
                        colliding = false;
                        break;
                    case USE_COLAVOID:
                        colliding = collision_mem_less();
                        break;
                    case USE_COLBACK:
                        colliding = collision();
                        break;
                    case STOP_ON_COLLISION:
                        colliding = collision();
                        break;
                    default:
                        colliding = false;
                        break;
                }

                if((bot.type == Model_Ground.Type.EXPLORE_AREA)|| (bot.type == Model_Ground.Type.RANDOM_MOVING_OBSTACLE)){
                    stage = null;
                    next = null;
                    colliding = true;
                }

                if(!colliding && stage != null) {
                    if(stage != prev)
                        gvh.log.e(TAG, "Stage is: " + stage.toString());
                    if(distance <= param.GOAL_RADIUS) {
                        next = STAGE.GOAL;
                    }
                    switch(stage) {
                        case INIT:
                            done = false;
                            if(mode == OPMODE.GO_TO) {
                                if(param.ENABLE_ARCING && distance <= param.ARC_RADIUS && absangle <= param.ARCANGLE_MAX) {
                                    next = STAGE.ARCING;
                                } else {
                                    next = STAGE.TURN;
                                }
                            } else {
                                next = STAGE.TURN;
                            }
                            break;
                        case ARCING:
                            // If this is the first run of ARCING, begin the arc
                            if(stage != prev) {
                                int radius = curveRadius();
                                curve(param.ARCSPEED_MAX, radius);
                            } else {
                                // Otherwise, check exit conditions
                                if(absangle > param.ARC_EXIT_ANGLE)
                                    next = STAGE.TURN;
                                if(absangle < param.STRAIGHT_ANGLE)
                                    next = STAGE.STRAIGHT;
                            }
                            break;
                        case STRAIGHT:
                            if(stage != prev) {
                                straight(LinSpeed(distance));
                            } else {
                                if(Common.inRange(distance, param.GOAL_RADIUS, param.SLOWFWD_RADIUS))
                                    straight(LinSpeed(distance));
                                if(Common.inRange(absangle, param.SMALLTURN_ANGLE, param.ARCANGLE_MAX))
                                    next = STAGE.SMALLTURN;
                                if(absangle > param.ARCANGLE_MAX)
                                    next = STAGE.TURN;
                            }
                            break;
                        case TURN:
                            if(stage != prev) {
                                gvh.log.e(TAG, "Sending turn command");
                                turn(TurnSpeed(absangle), angle);
                            } else {
                                if(absangle <= param.SMALLTURN_ANGLE) {
                                    gvh.log.i(TAG, "Turn stage: within angle bounds!");
                                    next = (mode == OPMODE.GO_TO) ? STAGE.STRAIGHT : STAGE.GOAL;
                                } else if(absangle <= param.SLOWTURN_ANGLE) {
                                    // Resend a reduced-speed turn command if we're
                                    // within the slow-turn window
                                    turn(TurnSpeed(absangle), angle);
                                }
                            }
                            break;
                        case SMALLTURN:
                            if(stage != prev) {
                                int radius = curveRadius() / 2;
                                curve(LinSpeed(distance), radius);
                            } else {
                                if(absangle <= param.SMALLTURN_ANGLE)
                                    next = STAGE.STRAIGHT;
                                if(absangle > param.ARCANGLE_MAX)
                                    next = STAGE.TURN;
                            }
                            break;
                        case GOAL:
                            done = true;
                            gvh.log.i(TAG, "At goal, done flag set!");
                            if(param.STOP_AT_DESTINATION)
                                straight(0);
                            running = false;
                            inMotion = false;
                            break;
                        case UNABLE:
                            System.out.println("motionautomation inactive");
                            gvh.log.i(TAG, "motion could not reach dest");
                            straight(0);
                            done = false;
                            running = false;
                            inMotion = false;
                            break;
                    }

                    prev = stage;
                    if(next != null) {
                        stage = next;
                        gvh.log.i(TAG, "Stage transition to " + stage.toString());
                        gvh.trace.traceEvent(TAG, "Stage transition", stage.toString(), gvh.time());
                    }
                    next = null;
                }

                if((colliding || stage == null) ) {
                    switch(param.COLAVOID_MODE) {
                        case USE_COLAVOID:
                            use_colavoid();
                            break;
                        case USE_COLBACK:
                            use_colback();
                            break;
                        case STOP_ON_COLLISION:
                            if(stage != null) {
                                gvh.log.d(TAG, "Imminent collision detected!");
                                straight(0);
                                stage = STAGE.UNABLE;
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            gvh.sleep(param.AUTOMATON_PERIOD);
        }
    }

    public abstract void cancel();

    // Ramp linearly from min at param.SMALLTURN_ANGLE to max at param.SLOWTURN_ANGLE
    private int TurnSpeed(int angle) {
        if(angle > param.SLOWTURN_ANGLE) {
            return param.TURNSPEED_MAX;
        } else if(angle > param.SMALLTURN_ANGLE && angle <= param.SLOWTURN_ANGLE) {
            return param.TURNSPEED_MIN + (int) ((angle - param.SMALLTURN_ANGLE) * turnspeed);
        } else {
            return param.TURNSPEED_MIN;
        }
    }

    @Override
    public void receivedKeyInput(String key){
        curKey = key;
    }

    protected void sendMotionEvent(int motiontype, int... argument) {
        // TODO: This might not be necessary
        gvh.trace.traceEvent(TAG, "Motion", Arrays.toString(argument), gvh.time());
        gvh.sendRobotEvent(RobotEventListener.Event.MOTION, motiontype);
    }

    protected abstract void curve(int velocity, int radius);

    protected abstract void straight(int velocity);

    protected abstract void turn(int velocity, int angle);

    private void use_colavoid() {
        if(stage != null) {
            gvh.log.d(TAG, "Imminent collision detected!");
            stage = null;
            straight(0);
            colnext = null;
            colprev = null;
            colstage = COLSTAGE.TURN;
        }
        switch(colstage) {
            case TURN:
                if(colstage != colprev) {
                    gvh.log.d(TAG, "Colliding: sending turn command");
                    turn(param.TURNSPEED_MAX, -1 * bot.angleTo(blocker.getPos()));

                }

                if(!colliding) {

                    colnext = COLSTAGE.STRAIGHT;

                } else {
                    gvh.log.d(TAG, "colliding with " + blocker.name + " - " + bot.isFacing(blocker) + " - " + bot.distanceTo(blocker));
                }
                break;
            case STRAIGHT:
                if(colstage != colprev) {
                    gvh.log.d(TAG, "Colliding: sending straight command");
                    straight(param.LINSPEED_MAX);
                    col_straightime = 0;
                } else {
                    col_straightime += param.AUTOMATON_PERIOD;
                    // If a collision is imminent (again), return to the
                    // turn stage
                    if(colliding) {
                        gvh.log.d(TAG, "Collision imminent! Cancelling straight stage");
                        straight(0);
                        colnext = COLSTAGE.TURN;
                    }
                    // If we're collision free and have been for enough
                    // time, restart normal motion
                    if(!colliding && col_straightime >= param.COLLISION_AVOID_STRAIGHTTIME) {
                        gvh.log.d(TAG, "Free! Returning to normal execution");
                        colprev = null;
                        colnext = null;
                        colstage = null;
                        stage = STAGE.INIT;
                    }
                }
                break;
        }
        colprev = colstage;
        if(colnext != null) {
            colstage = colnext;
            gvh.log.i(TAG, "Advancing stage to " + colnext);
        }
        colnext = null;
        return;
    }

    private void use_colback() {
        switch(bot.type) {
            case GET_TO_GOAL:
                goalbot();
                break;
            case EXPLORE_AREA:
                discoverbot();
                break;
            case RANDOM_MOVING_OBSTACLE:
                badbot();
                break;
            case ANTI_GOAL:
                break;
        }
    }

    private void goalbot() {
        if(stage != null) {
            gvh.log.d(TAG, "Imminent collision detected!");
            stage = null;
            straight(0);
            colnext0 = null;
            colstage0 = COLSTAGE0.BACK;
        }

        switch(colstage0) {
            case BACK:
                col_backtime += param.AUTOMATON_PERIOD;
                straight(- param.LINSPEED_MAX/2);
                if(col_backtime > param.COLLISION_AVOID_BACKTIME){
                    col_backtime = 0;
                    straight(0);
                    colnext0 = null;
                    colstage0 = null;
                    stage = STAGE.UNABLE;
                }
                break;
            case STRAIGHT:
                straight(param.LINSPEED_MAX);
                if(colliding) {
                    gvh.log.d(TAG, "Collision imminent! Cancelling straight stage");
                    straight(0);
                    colnext0 = COLSTAGE0.BACK;
                }
                break;
        }
        if(colnext0 != null) {
            colstage0 = colnext0;
            gvh.log.i(TAG, "Advancing stage to " + colnext);
        }
        colnext0 = null;

    }

    private void discoverbot() {
        if(stage != null) {
            gvh.log.d(TAG, "Imminent collision detected!");
            stage = null;
            straight(0);
            colnext1 = null;
            colstage1 = COLSTAGE1.BACK;
        }

        switch(colstage1) {
            case BACK:
                col_backtime += param.AUTOMATON_PERIOD;
                if (col_backtime > param.COLLISION_AVOID_BACKTIME){
                    col_backtime = 0;
                    straight(0);
                    colnext1 = COLSTAGE1.TURN;
                }
                else
                    straight(-param.LINSPEED_MAX/2);
                break;
            case STRAIGHT:
                col_straightime += param.AUTOMATON_PERIOD;
                if(!colliding){
                    if(col_straightime < param.COLLISION_AVOID_STRAIGHTTIME){
                        straight(param.LINSPEED_MAX);
                    }
                    else{
                        col_straightime = 0;
                        straight(0);
                        colnext1 = null;
                        colstage1 = null;
                        stage = STAGE.UNABLE;
                    }
                }
                else{
                    gvh.log.d(TAG, "Collision imminent! Cancelling straight stage");
                    straight(0);
                    col_straightime = 0;
                    colnext1 = COLSTAGE1.BACK;
                }
                break;
            case TURN:
                col_turntime += param.AUTOMATON_PERIOD;
                if(col_turntime < param.COLLISION_AVOID_TURNTIME)
                    turn(param.TURNSPEED_MAX, 45);
                    //trun left when hit
                else {
                    col_turntime = 0;
                    colnext1 = COLSTAGE1.SMALLARC;
                }
                break;
            case SMALLARC:
                col_straightime += param.AUTOMATON_PERIOD;
                if(!colliding){
                    if(col_straightime < param.COLLISION_AVOID_STRAIGHTTIME)
                        curve(param.LINSPEED_MAX , 320);
                    else {

                        colnext1 = COLSTAGE1.STRAIGHT;
                    }
                }
                else{
                    col_straightime = 0;
                    straight(0);
                    colnext1 = COLSTAGE1.BACK;
                }
                break;
        }
        if(colnext1 != null) {
            colstage1 = colnext1;
            gvh.log.i(TAG, "Advancing stage to " + colnext);
        }
        colnext1 = null;
    }

    private void badbot(){
        if(stage != null) {
            gvh.log.d(TAG, "Imminent collision detected!");
            stage = null;
            straight(0);
            colnext2 = null;
            colstage2 = COLSTAGE2.BACK;
        }
        switch(colstage2) {
            case BACK:
                col_backtime += param.AUTOMATON_PERIOD;
                if (col_backtime > param.COLLISION_AVOID_BACKTIME){
                    col_backtime = 0;
                    colstage2 = COLSTAGE2.RANDOM;
                }
                else{
                    straight(-param.LINSPEED_MAX/2);
                }
                break;
            case RANDOM:
                if(col_turntime == 0){
                    RanAngle = (int) (-90 + (Math.random()* 180 ));
                }
                col_turntime += param.AUTOMATON_PERIOD;
                if(col_turntime < param.COLLISION_AVOID_TURNTIME)
                    turn(param.TURNSPEED_MAX, RanAngle);
                    //trun left when hit
                else {
                    col_turntime = 0;
                    colnext2 = COLSTAGE2.STRAIGHT;
                }

                break;
            case STRAIGHT:
                if(colliding){
                    straight(0);
                    colnext2 = COLSTAGE2.BACK;
                }
                else{
                    straight(param.LINSPEED_MAX);
                }
                break;

        }
        if(colnext2 != null) {
            colstage2 = colnext2;
            gvh.log.i(TAG, "Advancing stage to " + colnext);
        }
        colnext2 = null;
    }

    // Calculates the radius of curvature to meet a target
    private int curveRadius() {
        int x0 = bot.getX();
        int y0 = bot.getY();
        int x1 = destination.getX();
        int y1 = destination.getY();
        int theta = (int) bot.angle;
        double alpha = -180 + Math.toDegrees(Math.atan2((y1 - y0), (x1 - x0)));
        double rad = -(Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2)) / (2 * Math.sin(Math.toRadians(alpha - theta))));
        return (int) rad;
    }

    private void startMotion() {
        running = true;
        stage = STAGE.INIT;
        inMotion = true;
    }

	/**
     * Slow down linearly upon coming within R_slowfwd of the goal
	 *
     * @param distance
	 * @return
     */
    private int LinSpeed(int distance) {
        if(distance > param.SLOWFWD_RADIUS)
            return param.LINSPEED_MAX;
        if(distance > param.GOAL_RADIUS && distance <= param.SLOWFWD_RADIUS) {
            return param.LINSPEED_MIN + (int) ((distance - param.GOAL_RADIUS) * linspeed);
        }
        return param.LINSPEED_MIN;
    }

    private boolean collision_mem_less(){
        if(bot.leftBump || bot.rightBump){
            double ColPoint_x, ColPoint_y;
            if(bot.leftBump && bot.rightBump){
                ColPoint_x = bot.radius()*(Math.cos(Math.toRadians(bot.angle))) + bot.getX();
                ColPoint_y = bot.radius()*(Math.sin(Math.toRadians(bot.angle))) + bot.getY();
                blocker = new ItemPosition("detected", (int) ColPoint_x, (int) ColPoint_y, 0);
            }
            else if(bot.leftBump){
                ColPoint_x = bot.radius()*(Math.cos(Math.toRadians(bot.angle+45))) + bot.getX();
                ColPoint_y = bot.radius()*(Math.sin(Math.toRadians(bot.angle+45))) + bot.getY();
                blocker = new ItemPosition("detected", (int) ColPoint_x, (int) ColPoint_y, 0);
            }
            else{
                ColPoint_x = bot.radius()*(Math.cos(Math.toRadians(bot.angle-45))) + bot.getX();
                ColPoint_y = bot.radius()*(Math.sin(Math.toRadians(bot.angle-45))) + bot.getY();
                blocker = new ItemPosition("detected", (int) ColPoint_x, (int) ColPoint_y, 0);
            }

            return true;
        }
        else
            return false;
    }

    private boolean collision() {
        boolean toreturn = collision_mem_less();
        if(toreturn)
            obsList.detected(blocker);
        return toreturn;
    }

    private void calcSpeeds() {
        linspeed = (param.LINSPEED_MAX - param.LINSPEED_MIN) / Math.abs(param.SLOWFWD_RADIUS - param.GOAL_RADIUS);
        turnspeed = (param.TURNSPEED_MAX - param.TURNSPEED_MIN) / (param.SLOWTURN_ANGLE - param.SMALLTURN_ANGLE);
    }

    protected enum STAGE {
        INIT, ARCING, STRAIGHT, TURN, SMALLTURN, GOAL, UNABLE
    }

    private enum OPMODE {
        GO_TO, TURN_TO
    }

    // Collision avoidance
    private enum COLSTAGE {
        TURN, STRAIGHT
    }

    // Detects an imminent collision with another robot or with any obstacles

    private enum COLSTAGE0 {
        BACK, STRAIGHT
    }

    private enum COLSTAGE1 {
        BACK, STRAIGHT, TURN, SMALLARC
    }

    private enum COLSTAGE2 {
        BACK, RANDOM, STRAIGHT
    }
}
