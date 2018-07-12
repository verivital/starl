package edu.illinois.mitra.demo.test;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;

import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.interfaces.RobotEventListener;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.motion.MotionParameters;
import edu.illinois.mitra.starl.motion.RRTNode;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PositionList;
/**
 * This app allows the user to set a waypoint by right clicking on the desired spot. Only one destination
 * can be chosen at a time. Can also change between piloting and automatic control by changing USER_CONTROL boolean.
 *
 * TODO: Re-implement userControl interface
 */


public class UserChooseApp extends LogicThread {
    private static final String TAG = "UserChoose App";
    private static final boolean USER_CONTROL = false;
    private int destIndex = 1;

    final Map<String, ItemPosition> destinations = new HashMap<String, ItemPosition>();
    ItemPosition currentDestination;

    //ReachAvoid is important, Robots won't move without calling
    PositionList<ItemPosition> doReachavoidCalls = new PositionList<ItemPosition>();

    //ObstacleList is important because it is needed when creating a gps to give the robot a view of the world,
    //even if no obstacles are loaded obs = gvh.gps.getObspointPositions(); needs to be called
    ObstacleList obs;

    //Deals with obstacle avoidance, implements RRT (rapidly exploring random tree) path finding algorithm using kd ( k-dimensional) tree
    public RRTNode kdTree;

    private enum Stage {
      GO, WAIT
    }


    private Stage stage = Stage.WAIT;


    public UserChooseApp(GlobalVarHolder gvh) {

        super(gvh);

        //MotionParameters object contains settings describing speeds and options to use in motion.
        MotionParameters.Builder settings = new MotionParameters.Builder();
        settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID);    //collision avoidance
        settings.GOAL_RADIUS(150);
        MotionParameters param = settings.build();

        gvh.plat.moat.setParameters(param);
        obs = gvh.gps.getObspointPositions();   //Gets Obstacle points
    }

    @Override
    public List<Object> callStarL() {
        while (true) {
            switch (stage) {
                case GO:
                    if (USER_CONTROL && gvh.plat.moat.done) {
                        if (currentDestination != null) {
                            destinations.remove(currentDestination.getName());
                        }
                        stage = Stage.WAIT;

                    } else if(gvh.plat.reachAvoid.doneFlag){
                        if (currentDestination != null) {
                            destinations.remove(currentDestination.getName());
                        }
                        stage = Stage.WAIT;
                    }
                    break;

                case WAIT:
                    if (!destinations.isEmpty()) {
                        currentDestination = destinations.get("Point " + destIndex);
                        destIndex++;
                        if(USER_CONTROL){
                            //gvh.plat.moat.userControl(currentDestination, obs);
                        } else {
                            kdTree = gvh.plat.reachAvoid.kdTree;
                            gvh.plat.reachAvoid.doReachAvoid(gvh.gps.getMyPosition(), currentDestination, obs);
                        }
                        stage = Stage.GO;
                    }
                    break;
            }
            sleep(100);

        }
    }

    /**
     * Receives a point where the user right clicked the mouse, then adds it to destinations
     * @param x coordinate
     * @param y coordinate
     */
    @Override
    public void receivedPointInput(int x, int y) {
        ItemPosition temp = new ItemPosition("Point " + destIndex, x, y);
        if(destinations.isEmpty()){
            destinations.put(temp.getName(), temp);
        }



    }


}

