package edu.illinois.mitra.demo.test;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;

import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.motion.MotionParameters;
import edu.illinois.mitra.starl.motion.RRTNode;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * App is set to either have two robots automatically go to unique points and wait for each other,
 * or it can be changed to enable to the user to pilot one robot using the arrow keys.
 * Change USER_CONTROl to true for piloting, and false for automatic movement based on ReachAvoid.
 *
 * TODO: Re-implement User Interface in zoomable panel and motion classes.
 */

public class TestApp extends LogicThread {
    private static final String TAG = "Test App";
    private static final boolean RANDOM_DESTINATION = false;
    private static final boolean USER_CONTROL = false;
    public static final int ARRIVED_MSG = 22;
    private boolean arrived = true;
    private int messageCount = 0;
    private int numBots;
    private int destIndex = 1;

    private HashSet<RobotMessage> receivedMsgs = new HashSet<RobotMessage>();   //keeps track of messages to ensure no duplicate wait mes.
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
        PICK, GO, DONE, FAIL, WAIT
    }

    //Start off by needing to pick a destination
    private Stage stage = Stage.PICK;


    public TestApp(GlobalVarHolder gvh) {

        super(gvh);

        //MotionParameters object contains settings describing speeds and options to use in motion.
        MotionParameters.Builder settings = new MotionParameters.Builder();
        settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID);    //collision avoidance
        settings.GOAL_RADIUS(150);
        MotionParameters param = settings.build();


        gvh.plat.moat.setParameters(param);

        /**
         *Destinations are loaded from Waypoint file, and are filtered and stored in hashmap for each robot based on unique key.
         *doReachAvoid directs robots to waypoints, must also be passed to TestDrawer.
         *
         * Keys: All numbers start at 1 and increment
         * iRob: #-iRob
         * quad: #-quad
         * All Others: #
         */

        int index = 1;
        for (ItemPosition i : gvh.gps.getWaypointPositions()) {
            if (gvh.plat.model instanceof Model_quadcopter) {
                if (i.getName().equals(index + "-quad")) {
                    destinations.put(i.getName(), i);
                    index++;
                }

            } else if (gvh.plat.model instanceof Model_iRobot) {
                if (i.getName().equals(index + "-iRob")) {
                    destinations.put(i.getName(), i);
                    index++;
                }

            } else {
                if (i.getName().equals(index + "")) {
                    destinations.put(i.getName(), i);
                    index++;
                }
            }
        }


        obs = gvh.gps.getObspointPositions();   //Gets Obstacle points
        numBots = gvh.id.getParticipants().size();
        gvh.comms.addMsgListener(this, ARRIVED_MSG);    //Used for receiving arrived messages.
    }

    @Override
    public List<Object> callStarL() {
        while (true) {
            switch (stage) {
                case PICK:
                    //Cycles through all destinations until they are empty
                    if (destinations.isEmpty()) {
                        stage = Stage.DONE;
                    } else {
                        currentDestination = getRandomElement(destinations, destIndex);
                        destIndex++;

                        if (!USER_CONTROL) {
                            kdTree = gvh.plat.reachAvoid.getKdTree();
                            gvh.plat.reachAvoid.doReachAvoid(gvh.gps.getMyPosition(), currentDestination, obs);
                        } else {
                            gvh.plat.moat.userControl(currentDestination, obs);
                        }

                        //Deals with log.
                        gvh.log.i("DoReachAvoid", currentDestination.getX() + " " + currentDestination.getY());
                        doReachavoidCalls.update(new ItemPosition(name + "'s " + "doReachAvoid Call to destination: " + currentDestination.name, gvh.gps.getMyPosition().getX(), gvh.gps.getMyPosition().getY()));
                        System.out.println(name + " going to " + currentDestination.getName());
                        stage = Stage.GO;

                    }
                    break;

                case GO:
                    //doneFlag is for reachAvoid algorithm, moat.done is for user control.
                    if (gvh.plat.reachAvoid.isDone() || (USER_CONTROL && gvh.plat.moat.done)) {
                        System.out.println("done");
                        if (currentDestination != null) {
                            destinations.remove(currentDestination.getName());
                        }
                        //gvh.log.i("DoneFlag", "read");
                        RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, currentDestination.getName());
                        arrived = true;
                        gvh.comms.addOutgoingMessage(inform);

                        stage = Stage.WAIT;
                    } else if (gvh.plat.reachAvoid.isFail()) {
                        //gvh.log.i("FailFlag", "read");
                        stage = Stage.FAIL;
                    }
                    break;

                case WAIT:
                    //Arrived message is sent, message Count is only incremented by other robots' messages
                    if ((messageCount >= numBots - 1) && arrived) {
                        messageCount = 0;
                        stage = Stage.PICK;
                    }
                    break;

                case FAIL:
                    //System.out.println(gvh.log.getLog());
                    break;
                case DONE:
                    //System.out.println(gvh.log.getLog());
                    return null;
            }
            sleep(100);
        }
    }

    @Override
//Each robot has instance of receive, they only receive others messages, not their own.
    protected void receive(RobotMessage m) {
        boolean alreadyReceived = false;
        for (RobotMessage msg : receivedMsgs) {
            if (msg.getFrom().equals(m.getFrom()) && msg.getContents().equals(m.getContents())) {
                alreadyReceived = true;
                break;
            }
        }
        if (m.getMID() == ARRIVED_MSG && !m.getFrom().equals(name) && !alreadyReceived) {
            gvh.log.d(TAG, "Adding to message count from " + m.getFrom());
            receivedMsgs.add(m);
            messageCount++;
        }
        //May not be necessary
        if ((messageCount == numBots) && arrived) {
            messageCount = 0;
            stage = Stage.PICK;
        }
    }

    private static final Random rand = new Random();

    @SuppressWarnings("unchecked")
    private <X, T> T getRandomElement(Map<X, T> map, int index) {
        if (RANDOM_DESTINATION)
            return (T) map.values().toArray()[rand.nextInt(map.size())];
        else {
            //Must change format of keys in .wpt to match the return.
            if (gvh.plat.model instanceof Model_quadcopter) {
                return (T) map.get(index + "-quad");
            } else if (gvh.plat.model instanceof Model_iRobot) {
                return (T) map.get(index + "-iRob");
            } else {
                return (T) map.get(index + "");
            }
        }
    }
}

