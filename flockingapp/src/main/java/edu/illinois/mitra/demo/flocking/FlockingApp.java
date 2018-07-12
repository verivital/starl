
/**
 * Created by Mousa Almotairi on 4/28/2015.
 */




package edu.illinois.mitra.demo.flocking;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;

import edu.illinois.mitra.starl.gvh.RobotGroup;

import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.motion.MotionParameters;

import edu.illinois.mitra.starl.objects.Common;

import edu.illinois.mitra.starl.objects.ItemPosition;

import edu.illinois.mitra.starl.functions.BarrierSynchronizer;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.Synchronizer;
import edu.illinois.mitra.starl.motion.RobotMotion;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * Created by Mousa Almotairi on 4/28/2015.
 * TODO: Fiz all num parse methods, set for iRobots currently. Once in formation, robots do not move smoothly
 */

public class FlockingApp extends LogicThread {
    private enum STAGE { START, SYNC, ELECT, MOVE, DONE }
    private STAGE stage = STAGE.START;


    private RobotMotion moat;


    private int n_waypoints;

    private static final boolean RANDOM_DESTINATION = false;


    final Map<String, ItemPosition> destinations = new HashMap<String, ItemPosition>();
    ItemPosition currentDestination;

    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    private int cur_waypoint = 0;
    PositionList pl = new PositionList();
    String wpn = "wp";

    private LeaderElection le;
    private Synchronizer sn;
    boolean initializeVee;

    public FlockingApp(GlobalVarHolder gvh) {
        super(gvh);

        initializeVee = true;


        //gvh.trace.traceStart();

        le = new RandomLeaderElection(gvh);



        le.elect();
        gvh.BotGroup = new RobotGroup(gvh.id.getName(), Common.numOFgroups);

        sn = new BarrierSynchronizer(gvh);

        moat = gvh.plat.moat;
        MotionParameters.Builder settings = new MotionParameters.Builder();
        settings = settings.ENABLE_ARCING(true);
        settings = settings.STOP_AT_DESTINATION(true);

        settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID); // buggy, just goes back, deadlocks...

        param = settings.build();
        moat.setParameters(param);
        //n_waypoints = gvh.gps.getWaypointPositions().getNumPositions();
        n_waypoints = Integer.MAX_VALUE;
        String n = wpn + gvh.id.getName() + cur_waypoint;
        pl.update(new ItemPosition(n, 2000, 2000, 0));

    }

    @Override
    public List<Object> callStarL() {
        String robotName = gvh.id.getName();
        Integer robotNum = Integer.parseInt(robotName.substring(6)); // assumes: botYYY
        Integer count = 0;
        Integer leaderNum = 1;
        boolean test = true;


        while(true) {
            switch (stage) {
                case START: {

                    sn.barrierSync("round" + count.toString());
                    stage = STAGE.SYNC;

                    System.out.printf("robot %3d, round " + count.toString() + "\n", robotNum);


                    break;
                }
                case SYNC: {


                    if (sn.barrierProceed("round" + count.toString())) {
                        stage = STAGE.ELECT;
                        // le.elect();

                    }




                 /*  */
                    break;
                }
                case ELECT: {
                    if(le.getLeader() != null) {
                        System.out.printf("robot %3d, leader is: " + le.getLeader() + "\n", robotNum);
                        leaderNum = Integer.parseInt(le.getLeader().substring(6)); // assumes: botYYY
                        stage = STAGE.MOVE;
                        // All below code in Elect state is to assign order-rank- for each robot in its group
                        if (robotNum!=leaderNum) {
                            if (gvh.BotGroup.setAfterBefore) {
                                ItemPosition myPosition = gvh.gps.getMyPosition();
                                int mySummation = myPosition.getX() + myPosition.getY();
                                int ranking = 1;
                                PositionList <ItemPosition> plAll = gvh.gps.get_robot_Positions();
                                for (ItemPosition rp : plAll.getList()) {
                                    Integer rpNum = Integer.valueOf(rp.getName().substring(6));
                                    if (rpNum != leaderNum) {
                                        if (rpNum != robotNum) {
                                            Integer rpGroup = Integer.valueOf(rp.getName().substring(6)) % Common.numOFgroups;
                                            if (gvh.BotGroup.getGroupNum() == rpGroup) {
                                                int otherSummation =  rp.getX() + rp.getY();
                                                // if (mySummation == otherSummation){

                                                //}
                                                if (mySummation == otherSummation)
                                                    System.out.println("############************** There is potential same locations ***************########### "+ robotName+" and "+ rp.getName());

                                                if (mySummation >= otherSummation) {
                                                    if (gvh.BotGroup.BeforeBot == null) {
                                                        if (mySummation == otherSummation) {
                                                            if (robotNum > Integer.valueOf(rp.getName().substring(6)))
                                                                gvh.BotGroup.BeforeBot = rp.getName();
                                                        }else
                                                            gvh.BotGroup.BeforeBot = rp.getName();

                                                    }
                                                    else {
                                                        int xSub = 0;
                                                        int ySub = 0;
                                                        // int angleSub = 0;
                                                        PositionList<ItemPosition> plAllSub = gvh.gps.get_robot_Positions();
                                                        for (ItemPosition rpSub : plAllSub.getList()) {
                                                            if (Integer.valueOf(rpSub.getName().substring(6)) == Integer.valueOf(gvh.BotGroup.BeforeBot.substring(6))) {
                                                                xSub = rpSub.getX();
                                                                ySub = rpSub.getY();
                                                                //angleSub = rpSub.angle;
                                                            }

                                                        }
                                                        int beforeBotSummation = xSub + ySub ;
                                                        if (otherSummation > beforeBotSummation)
                                                            gvh.BotGroup.BeforeBot = rp.getName();
                                                    }
                                                    if (mySummation == otherSummation){

                                                        System.out.println("############************** There is potential same locations ***************########### "+ robotName+" and "+ rp.getName());
                                                        if (robotNum<Integer.valueOf(rp.getName().substring(6))){
                                                            gvh.BotGroup.AfterBot = rp.getName();
                                                        }else{
                                                            gvh.BotGroup.BeforeBot = rp.getName();
                                                            ranking++;
                                                        }
                                                    }else
                                                        ranking++;
                                                } else if (mySummation < otherSummation)
                                                    if (gvh.BotGroup.AfterBot == null)
                                                        gvh.BotGroup.AfterBot = rp.getName();
                                                    else {
                                                        int xSub = 0;
                                                        int ySub = 0;
                                                        // int angleSub = 0;
                                                        PositionList<ItemPosition> plAllSub = gvh.gps.get_robot_Positions();
                                                        for (ItemPosition rpSub : plAllSub.getList()) {
                                                            if (Integer.valueOf(rpSub.getName().substring(6)) == Integer.valueOf(gvh.BotGroup.AfterBot.substring(6))) {
                                                                xSub = rpSub.getX();
                                                                ySub = rpSub.getY();
                                                                //angleSub = rpSub.angle;
                                                            }

                                                        }
                                                        int afterBotSummation = xSub + ySub ;

                                                        if (otherSummation == afterBotSummation)
                                                            if (robotNum<Integer.valueOf(rp.getName().substring(6)) )
                                                                gvh.BotGroup.AfterBot = rp.getName();

                                                        if (otherSummation < afterBotSummation)
                                                            gvh.BotGroup.AfterBot = rp.getName();
                                                    }
                                                else if (Integer.valueOf(gvh.id.getName().substring(6)) > Integer.valueOf(rp.getName().substring(6))) {
                                                    gvh.BotGroup.BeforeBot = rp.getName();
                                                    ranking++;
                                                } else gvh.BotGroup.AfterBot = rp.getName();
                                            }


                                        }
                                    }
                                }
                                gvh.BotGroup.rank = ranking;
                                gvh.BotGroup.setAfterBefore = false;
                                if (gvh.BotGroup.BeforeBot == null) {
                                    plAll = gvh.gps.get_robot_Positions();
                                    for (ItemPosition rp : plAll.getList())
                                        if (Integer.valueOf(rp.getName().substring(6)) == leaderNum)
                                            gvh.BotGroup.BeforeBot = rp.getName();
                                }
                                if (gvh.BotGroup.AfterBot == null)
                                    gvh.BotGroup.isLast = true;

                                Common.bots_neighbour[robotNum][0] = gvh.BotGroup.BeforeBot;
                                if (!gvh.BotGroup.isLast) Common.bots_neighbour[robotNum][1] = gvh.BotGroup.AfterBot;
                                else Common.bots_neighbour[robotNum][1] = "none";
                                Common.bots_neighbour[robotNum][2] = String.valueOf(gvh.BotGroup.rf);
                                gvh.BotGroup.setAfterBefore = false;
                            }
                        } else {
                            Common.bots_neighbour[robotNum][0] = "none";
                            Common.bots_neighbour[robotNum][1] = "none";
                            Common.bots_neighbour[robotNum][2] = "none";
                        }

                        /* For Testing purpose
                        for (int i=0; i<Common.numOFbots; i++){
                            System.out.println("bot"+i+" and his before bot is "+Common.bots_neighbour[i][0]+" and his after bot is "+Common.bots_neighbour[i][1]+" and group distance is "+Common.bots_neighbour[i][2]);
                        } */

                    }
                    break;
                }
                case MOVE: {
                    if(!moat.inMotion) {
                        //if(cur_waypoint < n_waypoints) {
                        //System.out.println(robotName + ": I've stopped moving!");
                        String n = wpn + gvh.id.getName() + cur_waypoint;

                        //System.out.println(robotName + ": New destination is (" + pl.getPosition(n).x + ", " + pl.getPosition(n).y + ")!");

                        ItemPosition dest;
                        if (initializeVee){



                            // Let the leader in the center
                            if (gvh.id.getName().equals(le.getLeader())){

                                dest = new ItemPosition(n, 0, 0, 0);

                                moat.goTo(dest);
                            }else{

                                // All other bots move to their place according to their order in the group

                                int oldX = gvh.BotGroup.rank*1000;
                                int oldY = 0;

                                double newXX = oldX*Math.cos(Math.toRadians(gvh.BotGroup.theta))-oldY*Math.sin(Math.toRadians(gvh.BotGroup.theta));
                                double newYY = oldY*Math.cos(Math.toRadians(gvh.BotGroup.theta))+oldX*Math.sin(Math.toRadians(gvh.BotGroup.theta));

                                int newX= (int) newXX;
                                int newY= (int) newYY;


                                dest = new ItemPosition(n, newX, newY, gvh.BotGroup.theta.intValue());

                                moat.goTo(dest);




                            }


                            initializeVee = false;
                        }
                        else {


                            // dest = new ItemPosition(n, newX, newY, gvh.BotGroup.theta.intValue());

                            //moat.goTo(dest);


                            //*********************** START: Rotation **********************
                            int newX=0;
                            int newY=0;
                            int beforeX=0;
                            int beforeY=0;
                            int afterX=0;
                            int afterY=0;
                            ItemPosition BeforeBot = new ItemPosition("BeforeBot", 0, 0, 0);
                            ItemPosition AfterBot = new ItemPosition("AfterBot", 0, 0, 0);


                            if (!gvh.id.getName().equals(le.getLeader())) {
                                PositionList<ItemPosition> plAll = gvh.gps.get_robot_Positions();
                                for (ItemPosition rp : plAll.getList()) {
                                    if (rp.getName().equals(gvh.BotGroup.BeforeBot))
                                        BeforeBot = rp;
                                    if (!gvh.BotGroup.isLast) {
                                        if (rp.getName().equals(gvh.BotGroup.AfterBot))
                                            AfterBot = rp;
                                    }

                                }
                            }


                            //  ****************** Rotation for the robot********************
                            double newXX = gvh.gps.getMyPosition().getX() * Math.cos(Math.toRadians(-gvh.BotGroup.theta)) - gvh.gps.getMyPosition().getY() * Math.sin(Math.toRadians(-gvh.BotGroup.theta));
                            double newYY = gvh.gps.getMyPosition().getY() * Math.cos(Math.toRadians(-gvh.BotGroup.theta)) + gvh.gps.getMyPosition().getX() * Math.sin(Math.toRadians(-gvh.BotGroup.theta));

                            newX = (int) newXX;
                            newY = (int) newYY;

                            //******************** Rotation for robot before the Robot (Left Robot)**************
                            double beforeXX = BeforeBot.getX() * Math.cos(Math.toRadians(-gvh.BotGroup.theta)) - BeforeBot.getY() * Math.sin(Math.toRadians(-gvh.BotGroup.theta));
                            double beforeYY = BeforeBot.getY() * Math.cos(Math.toRadians(-gvh.BotGroup.theta)) + BeforeBot.getX() * Math.sin(Math.toRadians(-gvh.BotGroup.theta));

                            beforeX = (int) beforeXX;
                            beforeY = (int) beforeYY;

                            //******************** Rotation for robot after the Robot (right Robot)**************
                            double afterXX = AfterBot.getX() * Math.cos(Math.toRadians(-gvh.BotGroup.theta)) - AfterBot.getY() * Math.sin(Math.toRadians(-gvh.BotGroup.theta));
                            double afterYY = AfterBot.getY() * Math.cos(Math.toRadians(-gvh.BotGroup.theta)) + AfterBot.getX() * Math.sin(Math.toRadians(-gvh.BotGroup.theta));


                            afterX = (int) afterXX;
                            afterY = (int) afterYY;


                            //*********************** END: Rotation   **********************

                            //*********************** START: Forming the flock **********************
                            //*********************** Leader doesn't need any change
                            if (!gvh.id.getName().equals(le.getLeader())) {


                             //*********************** If Robot is the Rightmost (Last robot in the group)
                                if (gvh.BotGroup.isLast) {


                                    newX = (beforeX + newX + gvh.BotGroup.rf) / 2;
                                    newY = (beforeY + newY) / 2;

                                } else {

                                    // ******************** If it is interior


                                    newX = (beforeX + afterX) / 2;
                                    newY = (beforeY + afterY) / 2;

                                }
                            }

                            //*********************** END: Forming the flock   **********************


                            //*********************** START: Rotation Back**********************

                            //*********************** Leader doesn't need any change
                            // if (!gvh.id.getName().equals(le.getLeader())) {

                            if (is_Flocking()) {
                                gvh.BotGroup.theta = gvh.BotGroup.theta + 20;

                                newX= newX+100;
                                newY= newY+150;
                                //gvh.BotGroup.rf *= 1.25;

                               /* System.out.println("Robot number is "+ robotNum);
                                if (!Common.bots_neighbour[robotNum][2].equals("none")) {
                                    Common.bots_neighbour[robotNum][2] = String.valueOf(gvh.BotGroup.rf);
                                }*/


                            }

                            //  System.out.println("Back Angle: " + robotName + " its new X coordinate is " + gvh.BotGroup.theta);


                            newXX = newX * Math.cos(Math.toRadians(gvh.BotGroup.theta)) - newY * Math.sin(Math.toRadians(gvh.BotGroup.theta));
                            newYY = newY * Math.cos(Math.toRadians(gvh.BotGroup.theta)) + newX * Math.sin(Math.toRadians(gvh.BotGroup.theta));

                            newX = (int) newXX;
                            newY = (int) newYY;

                            //    System.out.println("Back Rotation: " + robotName + " its new X coordinate is " + newX + " and its new Y coordinate is " + newY + " and its order in groups is " + gvh.BotGroup.rank);
                            // }

                            //*********************** END: Rotation   **********************


                            System.out.println(robotName+" has old coordination X "+gvh.gps.getMyPosition().getX()+" and Y "+gvh.gps.getMyPosition().getY()+" New X is "+newX+" and New Y is "+newY);
                            dest = new ItemPosition(n, newX, newY, gvh.BotGroup.theta.intValue());

                            moat.goTo(dest);
                        }


                        count += 1;

                    }

                    // wait here while robot is in motion
                    while (moat.inMotion) {
                        gvh.sleep(100);
                    }

                    stage = STAGE.START; // repeat

                    break;
                }

                case DONE: {
                    gvh.trace.traceEnd();
                    return Arrays.asList(results);
                }
            }
            gvh.sleep(100);

        }
    }

    /*
	@Override
	protected void receive(RobotMessage m) {
		String posName = m.getContents(0);
		if(destinations.containsKey(posName))
			destinations.remove(posName);

		if(currentDestination.getName().equals(posName)) {
			gvh.plat.moat.cancel();
			stage = Stage.PICK;
		}
	}*/

    private static final Random rand = new Random();

    @SuppressWarnings("unchecked")
    private <X, T> T getRandomElement(Map<X, T> map) {
        if(RANDOM_DESTINATION)
            return (T) map.values().toArray()[rand.nextInt(map.size())];
        else
            return (T) map.values().toArray()[0];
    }
    public boolean is_Flocking ()
    {
        boolean isFlocking = true;

        ItemPosition BeforeBot = new ItemPosition("BeforeBot", 0, 0, 0);
        ItemPosition AfterBot = new ItemPosition("AfterBot", 0, 0, 0);
        ItemPosition Bot = new ItemPosition("Bot", 0, 0, 0);

        int groupDis = 0;

        Integer leadNum = Integer.valueOf(le.getLeader().substring(6));

        boolean once = true;
        for  (int i=0; i<Common.numOFbots; i++) {
            if (!Common.bots_neighbour[i][2].equals("none")) {
                groupDis = Integer.parseInt(Common.bots_neighbour[i][2]);

                if (once ) {
                    System.out.println("Reference distance between each group is " +groupDis);
                    once =false;
                }

            }
            if (i !=leadNum) {
                PositionList<ItemPosition> plAll = gvh.gps.get_robot_Positions();
                for (ItemPosition rp : plAll.getList()) {
                    if (rp.getName().equals("bot"+i))
                        Bot = rp;
                    if (rp.getName().equals(Common.bots_neighbour[i][0]))
                        BeforeBot = rp;
                    if (!Common.bots_neighbour[i][1].equals("none")) {
                        if (rp.getName().equals(Common.bots_neighbour[i][1]))
                            AfterBot = rp;
                    }

                }

                // Distance between the bot and his before (left) neighbour
                double botDistance = Math.sqrt((Bot.getX()-BeforeBot.getX())*(Bot.getX()-BeforeBot.getX())+(Bot.getY()-BeforeBot.getY())*(Bot.getY()-BeforeBot.getY()));
                if (botDistance < (groupDis-groupDis*0.3)  || botDistance > (groupDis+groupDis*0.3) ) {

                    System.out.println("It is false because before bot is out of the range, their distance between each other is "+ String.valueOf(botDistance));
                    return false;
                }

                // Distance between the bot and his after (right) neighbour
                if (!Common.bots_neighbour[i][1].equals("none")) {
                    double botDistanceAfter = Math.sqrt((Bot.getX()-AfterBot.getX())*(Bot.getX()-AfterBot.getX())+(Bot.getY()-AfterBot.getY())*(Bot.getY()-AfterBot.getY()));

                    if (botDistanceAfter < (groupDis-groupDis*0.3) || botDistanceAfter > (groupDis+groupDis*0.3)) {
                        System.out.println("It is false because after bot is out of the range, their distance between each other is "+ String.valueOf(botDistance));
                        return false;
                    }
                }
            }
        }


        return isFlocking;
    }
}
