package edu.illinois.mitra.demo.arrow;

//Written by Lucas Buccafusca
//05-31-2012
//What the App does:
//At the beginning of implementation, the robots are synced together to allow for communication between them.
//A leader is chosen through a determined leader selection (bot0 will always be the leader)
//The robots will then travel (while maintaining the arrow shape) to a series of waypoints

//App not currently working, simplistic flocking formation based on going to waypoints without any
//neighbour monitoring.

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;

import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.functions.BarrierSynchronizer;
import edu.illinois.mitra.starl.functions.PickedLeaderElection;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.interfaces.MessageListener;
import edu.illinois.mitra.starl.interfaces.Synchronizer;
import edu.illinois.mitra.starl.motion.MotionParameters;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PositionList;
import edu.illinois.mitra.starlSim.main.SimSettings;



public class ArrowTravelApp extends LogicThread implements MessageListener {

    SortedSet<ItemPosition> toVisit = new TreeSet<ItemPosition>();
    SortedSet<String> toVisit2 = new TreeSet<String>();
    SortedSet<String> arrived = new TreeSet<String>();

    final Map<String, ItemPosition> destinations = new HashMap<String, ItemPosition>();
    ItemPosition currentDestination;

    private static final String TAG = "Logic";

    private ItemPosition destname = null;
    private String leader = null;

    private boolean running = true;

    private boolean iamleader = false;
    private SimSettings.Builder settings = new SimSettings.Builder();
    private LeaderElection le;
    private Synchronizer sync;
    private int d_r=700; //Some distance that each robot will be from the nearest robot
    private double theta_r=Math.PI/4; //0<theta_r<2*PI, theta_r != PI/2, 3*PI/2... Be wary of angles <PI/2.7

    String robotName = gvh.id.getName();
    Integer robotNum = gvh.id.getIdNumber();


    private final static String SYNC_START = "1";


    private enum STAGE { START, SYNC, LE, MOVE,WAYPOINT_CALC, WAYPOINT_TRAVEL, WAIT_TO_ARRIVE, DONE };
    private STAGE setup= STAGE.START;
    private STAGE stage = STAGE.START;

    public ArrowTravelApp(GlobalVarHolder gvh) {
        super(gvh);


        // Get the list of position to travel to
        for(ItemPosition ip : gvh.gps.getWaypointPositions().getList()) {
            toVisit.add(ip);
            destinations.put(ip.getName(),ip);
        }

        MotionParameters.Builder settings = new MotionParameters.Builder();
        settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID); // buggy, just goes back, deadlocks...
        MotionParameters param = settings.build();
        gvh.plat.moat.setParameters(param);

        // Progress messages are broadcast with message ID 99
        gvh.comms.addMsgListener(this,99);

        // Make sure waypoints were provided
        if(gvh.gps.getWaypointPositions().getNumPositions() == 0) System.out.println("This application requires waypoints to travel to!");

        sync = new BarrierSynchronizer(gvh);
        le = new PickedLeaderElection(gvh);




    }


    @Override
    public List<Object> callStarL() {

        //Declares leader
        while (running){
            gvh.sleep(100);
            gvh.plat.setDebugInfo(gvh.id.getParticipants().toString());
            switch (setup) {
                case START:
                    sync.barrierSync(SYNC_START);
                    setup = STAGE.SYNC;
                    gvh.log.d(TAG, "Syncing...");
                    System.out.println("Syncing..." + name);
                    PositionList<ItemPosition> plAll = gvh.gps.get_robot_Positions();
                    for (ItemPosition rp : plAll) {
                        arrived.add(rp.getName());
                        System.out.println(arrived + " " + gvh.id.getName());
                    }
                    break;
                case SYNC:
                    if (sync.barrierProceed(SYNC_START)) {
                        setup = STAGE.LE;
                        le.elect();
                        gvh.log.d(TAG, "Synced!");
                    }
                    break;
                case LE:
                    if(le.getLeader() != null) {
                        gvh.log.d(TAG, "Electing...");
                        leader = le.getLeader();
                        iamleader = leader.equals(name);
                        System.out.println("Robot Leader? "+leader);
                        setup = STAGE.MOVE;

                    }
                    break;

                case MOVE:

                    ItemPosition startPoint = startpoint();
                    System.out.println(startPoint + " " + gvh.id.getName());
                    destinations.put(startPoint.getName(),startPoint);
                    gvh.plat.moat.goTo(startPoint);
                    boolean motionSuccess = true;
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(100);
                        if(!toVisit2.contains("START POINT" + robotNum)) {
                            motionSuccess = false;
                            break;
                        }
                    }

                    // If Arrival of the LAST robot
                    if(motionSuccess && toVisit2.isEmpty()) {
                        setup = STAGE.MOVE;

                    }
                    else if (!toVisit2.isEmpty()) {

                        toVisit2.remove("START POINT" + robotNum);
                    }

                    if(toVisit2.isEmpty()) {
                        setup = STAGE.WAIT_TO_ARRIVE;
                    } else {
                        setup = STAGE.MOVE;
                    }
                    break;

                case WAIT_TO_ARRIVE:

                    motionSuccess = true;
                    while(gvh.plat.moat.inMotion) {
                        gvh.sleep(10);
                        if(!toVisit2.contains(destname.getName())) {
                            motionSuccess = false;
                            break;
                        }
                    }

                    // If Arrives
                    if(motionSuccess) {
                        RobotMessage inform = new RobotMessage("ALL", name, 99, "bot"+robotNum);
                        gvh.comms.addOutgoingMessage(inform);
                        arrived.remove(gvh.id.getName());
                        setup = STAGE.WAYPOINT_CALC;
                    }


                    setup = STAGE.WAYPOINT_CALC;


                    break;

                case WAYPOINT_CALC:
                    //Calculation of waypoints for other robots
                    if (arrived.isEmpty()) {
                        newpoint();
                        setup=STAGE.WAYPOINT_TRAVEL;
                        break;}
                    else{
                        setup=STAGE.WAYPOINT_CALC;
                        gvh.sleep(10);

                    }
                    break;


                case WAYPOINT_TRAVEL:
                    gvh.sleep(10);
                    gvh.plat.moat.turnTo(newpoint());
                    gvh.sleep(2100);
                    if (arrived.isEmpty())
                    {

                        ItemPosition newPoint = newpoint();
                        destinations.put(newPoint.getName(),newPoint);
                        gvh.plat.moat.goTo(newPoint);
                        while (gvh.plat.moat.inMotion)
                        {
                            gvh.sleep(1);
                        }
                        toVisit.remove(toVisit.first());


                    }
                    if(toVisit.isEmpty()) {
                        setup = STAGE.DONE;
                        break;
                    } else {
                        setup = STAGE.WAYPOINT_TRAVEL;
                        break;
                    }



                case DONE:
                    System.out.println(name + " is done.");
                    gvh.plat.moat.motion_stop();
                    return Arrays.asList(results);

            }



        }
        return null;
    }


    @Override
    public void receive(RobotMessage m) {
        synchronized(arrived) {
            // Remove the received waypoint from the list of waypoints to visit
            arrived.remove(m.getContents(0));

        }

        synchronized(stage) {
            // If no waypoints remain, quit. Otherwise, go on to the next destination
            if(arrived.isEmpty()) {
                stage = STAGE.DONE;
            } else {
                stage = STAGE.MOVE;
            }

        }
    }

    private ItemPosition startpoint() {

        String robotName = gvh.id.getName();
        Integer robotNum = gvh.id.getIdNumber();
        toVisit2.add("START POINT" + robotNum);
        if(iamleader){
            return new ItemPosition("goHere",settings.getGRID_XSIZE()/2, settings.getGRID_YSIZE()/2, 0);
        }
        else {
            if(robotNum == 0){
                robotNum = le.getLeaderID();
            }
            if (robotNum % 2 == 0 && !iamleader) {
                return new ItemPosition("goHere",(int) (settings.getGRID_XSIZE()/2 +d_r*robotNum/2*Math.cos(theta_r)), (int) (settings.getGRID_YSIZE()/2 -d_r*robotNum/2*Math.sin(theta_r)), 0);
            }

            if (robotNum % 2 == 1) {
                return new ItemPosition("goHere",(int) (settings.getGRID_XSIZE()/2 -d_r*(robotNum+1)/2*Math.cos(theta_r)), (int) (settings.getGRID_YSIZE()/2 -d_r*(robotNum+1)/2*Math.sin(theta_r)), 0);
            }
            else
            {return null;}

        }
    }
    private ItemPosition newpoint() {

        String robotName = gvh.id.getName();
        Integer robotNum = Integer.parseInt(robotName.substring(6)); // assumes: botYYY
        destname = toVisit.first();

        if(iamleader){

            return new ItemPosition("goHere", destname.getX(), destname.getY(), 0);
        }
        else {
            if(robotNum == 0){
                robotNum = le.getLeaderID();
            }
            if (robotNum % 2 == 0 && !iamleader)
            {return new ItemPosition("goHere",(int)(destname.getX() + d_r*robotNum/2*Math.cos(theta_r)), (int) (destname.getY() -d_r*robotNum/2*Math.sin(theta_r)), 0);
            }

            if (robotNum % 2 == 1)
            {return new ItemPosition("goHere",(int)(destname.getX() -d_r*(robotNum+1)/2*Math.cos(theta_r)), (int) (destname.getY() -d_r*(robotNum+1)/2*Math.sin(theta_r)), 0);
            }
            else
            {return null;}

        }
    }






}