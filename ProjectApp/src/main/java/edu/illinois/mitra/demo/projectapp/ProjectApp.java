package edu.illinois.mitra.demo.projectapp;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Stack;
import java.util.Vector;
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.*;
import edu.illinois.mitra.starl.motion.*;
import edu.illinois.mitra.starl.motion.MotionParameters.COLAVOID_MODE_TYPE;

public class ProjectApp extends LogicThread {
    private static final boolean RANDOM_DESTINATION = false;
    public static final int ARRIVED_MSG = 22;
    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    // this is an ArrayList of HashMap. Each HashMap element in the array will contain one set of waypoints
    final ArrayList<HashMap<String, ItemPosition>> destinations = new ArrayList<>();
    private int numSetsWaypoints = 4;
    int robotIndex;
    int currentSet;
    boolean goingToRandom;
    Random random = new Random();


    // used to find path through obstacles
    Stack<ItemPosition> pathStack;
    RRTNode kdTree = new RRTNode();

    ObstacleList obsList;
    //obsList is a local map each robot has, when path planning, use this map
    ObstacleList obEnvironment;
    ObstacleList obsRobots;
    //obEnvironment is the physical environment, used when calculating collisions

    ItemPosition currentDestination, preDestination;

    private LeaderElection le;
    //	private String leader = null;
    private boolean iamleader = false;

    private enum Stage {
        PICK, WAIT, DONE, ELECT, HOLD, MIDWAY, PICKRANDOM, PLAN
    };

    private Stage stage = Stage.ELECT;

    public ProjectApp(GlobalVarHolder gvh) {
        super(gvh);

        obsRobots = new ObstacleList();

        for(int i = 0; i< gvh.gps.get_robot_Positions().getNumPositions(); i++){
            if(gvh.gps.get_robot_Positions().getList().get(i).name == name){
                robotIndex = i;
                break;
            }

            int iX = gvh.gps.get_robot_Positions().getList().get(i).getX();
            int iY = gvh.gps.get_robot_Positions().getList().get(i).getY();
            Obstacles tmpO = new Obstacles();
            tmpO.add(iX, iY);
            obsRobots.ObList.add(tmpO);
        }

        // instantiates each HashMap object in the array
        for(int i = 0; i < numSetsWaypoints; i++) {
            destinations.add(new HashMap<String, ItemPosition>());
        }
        le = new RandomLeaderElection(gvh);


        MotionParameters.Builder settings = new MotionParameters.Builder();
//		settings.ROBOT_RADIUS(400);
        settings.COLAVOID_MODE(COLAVOID_MODE_TYPE.USE_COLBACK);
        //settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID);
        MotionParameters param = settings.build();
        gvh.plat.moat.setParameters(param);

        // this loop gets add each set of waypoints i to the hashmap at destinations(i)
        for(ItemPosition i : gvh.gps.getWaypointPositions()) {
            String setNumStr = i.getName().substring(0,1);
            int setNum = Integer.parseInt(setNumStr);
            destinations.get(setNum).put(i.getName(), i);
        }


        //point the environment to internal data, so that we can update it
        obEnvironment = gvh.gps.getObspointPositions();

        //download from environment here so that all the robots have their own copy of visible ObstacleList
        obsList = gvh.gps.getViews().elementAt(robotIndex) ;

        gvh.comms.addMsgListener(this, ARRIVED_MSG);
    }

    @Override
    public List<Object> callStarL() {
        // assume environment is closed: find maximum positions of obstacles in environment
        int environmentMinX = Integer.MAX_VALUE;
        int environmentMaxX = Integer.MIN_VALUE;
        int environmentMinY = Integer.MAX_VALUE;
        int environmentMaxY = Integer.MIN_VALUE;
        int environmentXSize;
        int environmentYSize;

        for (Obstacles o : obEnvironment.ObList) {
            for (Point3i obstaclePoints : o.getObstacleVector()) {
                environmentMinX = (int)Math.min(obstaclePoints.getX(), environmentMinX);
                environmentMaxX = (int)Math.max(obstaclePoints.getX(), environmentMaxX);
                environmentMinY = (int)Math.min(obstaclePoints.getY(), environmentMinY);
                environmentMaxY = (int)Math.max(obstaclePoints.getY(), environmentMaxY);
            }
        }
        environmentXSize = environmentMaxX - environmentMinX;
        environmentYSize = environmentMaxY - environmentMinY;

        currentSet = 0;
        while(true) {

            /*
            //  try to plan around other robots: this doesn't work, please fix it, we need to give an example of this as it's apparently nontrivial
            for(int i = 0; i< gvh.gps.get_Robot_Positions().getNumPositions(); i++){


            //int myX = gvh.gps.get_robot_Positions().getList().get(robotIndex).getX();
            //int myY = gvh.gps.get_robot_Positions().getList().get(robotIndex).getX();

            //int otherX = gvh.gps.get_robot_Positions().getList().get(i).getX();
            //int otherY = gvh.gps.get_robot_Positions().getList().get(i).getX();

                int otherX = gvh.gps.get_robot_Positions().getList().get(i).getX();
                int otherY = gvh.gps.get_robot_Positions().getList().get(i).getY();

                int radius = (gvh.gps.get_robot_Positions().getList().get(i).radius);

                //if (Point.distance(myX, myY, otherX, otherY)


                if (i != robotIndex) {
                    //obsList.detected(gvh.gps.get_robot_Positions().getList().get(i));
                    Obstacles oTmp = new Obstacles(otherX, otherY);


                    //oTmp.add(otherX + radius, otherY);
                    //oTmp.add(otherX - radius, otherY);
                    //oTmp.add(otherX, otherY + radius);
                    //oTmp.add(otherX, otherY - radius);

                    oTmp.timeFrame = System.currentTimeMillis() + 10000;
                    //obsList.addObstacle(oTmp);
                    obEnvironment.addObstacle(oTmp);
                }

            }
        */


            obEnvironment.updateObs();
            obsList.updateObs();

            if(destinations.get(currentSet).isEmpty()) {
                if(currentSet+1 >= numSetsWaypoints) {
                    stage = Stage.DONE;
                }
                else {
                    currentSet++;
                    obsList.ObList.clear();
                    stage = Stage.ELECT;
                }

            }

            if((((Model_Ground)gvh.plat.model).type == Model_Ground.Type.GET_TO_GOAL)
                    || (((Model_Ground)gvh.plat.model).type == Model_Ground.Type.EXPLORE_AREA)){


                switch(stage) {
                    case ELECT:
					    if(gvh.id.getIdNumber() == 0) {
                            stage = Stage.PICK;
                        }
                        else
                        {
                            stage = Stage.PICKRANDOM;
                        }

                        break;
                    case PICK:
                        /*if(destinations.get(currentSet).isEmpty()) {
                            if(currentSet+1 >= numSetsWaypoints) {
                                stage = Stage.DONE;
                            }
                            else {
                                currentSet++;
                            }
                        } else
                        {*/

                            //			RobotMessage informleader = new RobotMessage("ALL", name, 21, le.getLeader());
                            //			gvh.comms.addOutgoingMessage(informleader);

                            //			iamleader = le.getLeader().equals(name);
                                if(gvh.plat.moat.inMotion) {
                                    gvh.plat.moat.motion_stop();
                                }
                                currentDestination = getRandomElement(destinations.get(currentSet));
                                goingToRandom = false;
                                stage = Stage.PLAN;
                                break;

                    case PICKRANDOM:
                                goingToRandom = true;
                                int x = random.nextInt(environmentXSize);
                                int y = random.nextInt(environmentYSize);
                                currentDestination = new ItemPosition("random", x, y, 0);
                                stage = Stage.PLAN;
                                break;
                    case PLAN:

                                RRTNode path = new RRTNode(gvh.gps.getPosition(name).getX(), gvh.gps.getPosition(name).getY());
                                // TODO: change this dynamically based on current obstacles
                                // obEnvrinoment vs. obsList

                                obsList.updateObs();
                                obEnvironment.updateObs();
                                ObstacleList allObstacles = new ObstacleList();
                                //allObstacles.addObstacles(obsList.ObList);

                                // how to add all the current robot positions to plan around these?
                                allObstacles.addObstacles(obEnvironment.ObList);

                                for(int i = 0; i< gvh.gps.get_robot_Positions().getNumPositions(); i++){


                                    //int myX = gvh.gps.get_robot_Positions().getList().get(robotIndex).getX();
                                    //int myY = gvh.gps.get_robot_Positions().getList().get(robotIndex).getX();

                                    //int otherX = gvh.gps.get_robot_Positions().getList().get(i).getX();
                                    //int otherY = gvh.gps.get_robot_Positions().getList().get(i).getX();

                                    int otherX = gvh.gps.get_robot_Positions().getList().get(i).getX();
                                    int otherY = gvh.gps.get_robot_Positions().getList().get(i).getY();

                                    int radius = gvh.plat.model.radius();

                                    //if (Point.distance(myX, myY, otherX, otherY)

                                    // only add other robots
                                    if (i != robotIndex) {
                                        //obsList.detected(gvh.gps.get_robot_Positions().getList().get(i));
                                        Obstacles oTmp = new Obstacles(otherX, otherY);


                                        oTmp.add(otherX + radius, otherY);
                                        oTmp.add(otherX - radius, otherY);
                                        oTmp.add(otherX, otherY + radius);
                                        oTmp.add(otherX, otherY - radius);

                                        oTmp.timeFrame = System.currentTimeMillis() + 10000;
                                        //obsList.addObstacle(oTmp);
                                        allObstacles.addObstacle(oTmp);
                                    }
                                }

                                allObstacles.updateObs();


                                //// TODO: Enter correct parameters to pathStack
                                // TODO: want to use smallest size in RRT search as possible (otherwise, the found path may be very large)
                                // one strategy: start small, try to find, if it fails, double x, and y sizes?
                                // TODO: perform re-planning when en route to the target? e.g., in the Stage.MIDWAY state?
                                pathStack = path.findRoute(currentDestination, 50, allObstacles, environmentMinX, environmentMaxX, environmentMinY,environmentMaxY,(gvh.gps.getPosition(name)), (int)(gvh.plat.model.radius()));
                                kdTree = RRTNode.stopNode;

                                if (pathStack != null) {
                                    preDestination = null;
                                    stage = Stage.MIDWAY;
                                }

                                if (pathStack == null && goingToRandom) {
                                    stage = Stage.PICKRANDOM;
                                }


						/*
						else
						{
						currentDestination = gvh.gps.getPosition(le.getLeader());
						currentDestination1 = new ItemPosition(currentDestination);
						int newx, newy;
						if(gvh.gps.getPosition(name).getX() < currentDestination1.getX())
						{
							newx = gvh.gps.getPosition(name).getX() - currentDestination1.getX()/8;
						}
						else
						{
							newx = gvh.gps.getPosition(name).getX() + currentDestination1.getX()/8;
						}
						if(gvh.gps.getPosition(name).getY() < currentDestination1.getY())
						{
							newy = gvh.gps.getPosition(name).getY() - currentDestination1.getY()/8;
						}
						else
						{
							newy = gvh.gps.getPosition(name).getY() + currentDestination1.getY()/8;
						}
						currentDestination1.setPos(newx, newy, (currentDestination1.getAngle()));
		//				currentDestination1.setPos(currentDestination);
						gvh.plat.moat.goTo(currentDestination1, obsList);
						stage = Stage.HOLD;
						}
						*/
                        //}
                        break;


                    case MIDWAY:
                        if(!gvh.plat.moat.inMotion) {
                            if(pathStack == null){
                                stage = Stage.HOLD;
                                // if can not find a path, wait for obstacle map to change
                                break;
                            }
                            if(!pathStack.empty()){
                                //if did not reach last midway point, go back to path planning
                                if(preDestination != null){
                                    if((gvh.gps.getPosition(name).distanceTo(preDestination)>param.GOAL_RADIUS)){
                                        pathStack.clear();
                                        if(!goingToRandom) {
                                            stage = Stage.PICK;
                                        }
                                        else {stage = Stage.PICKRANDOM;}
                                        break;
                                    }
                                    preDestination = pathStack.peek();
                                }
                                else{
                                    preDestination = pathStack.peek();
                                }
                                ItemPosition goMidPoint = pathStack.pop();
                                gvh.plat.moat.goTo(goMidPoint, obsList);
                                stage = Stage.MIDWAY;
                            }
                            else{
                                if((gvh.gps.getPosition(name).distanceTo(currentDestination)>param.GOAL_RADIUS)){
                                    pathStack.clear();
                                    if(!goingToRandom) {
                                        stage = Stage.PICK;
                                    }
                                    else {stage = Stage.PICKRANDOM;}
                                }
                                else{
                                    if(currentDestination != null){
                                      if(!goingToRandom) {
                                          destinations.get(currentSet).remove(currentDestination.getName());
                                          RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, currentDestination.getName());
                                          gvh.comms.addOutgoingMessage(inform);
                                          // if this is the highest numbered bot and there are more points, go to pick
                                          if (gvh.id.getIdNumber() == gvh.id.getParticipants().size() - 1 && !destinations.get(currentSet).isEmpty()) {
                                              stage = Stage.PICK;
                                          } else {
                                              stage = Stage.PICKRANDOM;
                                          }
                                      }
                                        else {
                                          stage = Stage.PICKRANDOM;
                                      }
                                    }
                                }
                            }
                        }
                        break;

                    case WAIT:
                        if(destinations.get(currentSet).isEmpty()) {
                            if(currentSet+1 >= numSetsWaypoints) {
                                stage = Stage.DONE;
                            }
                            else {
                                currentSet++;
                                obsList.ObList.clear();
                                stage = Stage.ELECT;
                            }

                        }


                        break;
                    case HOLD:
                        //			if(gvh.gps.getMyPosition().distanceTo(gvh.gps.getPosition(le.getLeader())) < 1000 )
                        //			{
                        //			stage = Stage.PICK;
                        //		    }
                        //			else
                    {

                        gvh.plat.moat.motion_stop();
                        gvh.plat.moat.cancel();
                        obsList.ObList.clear();
                        //destinations.get(currentSet).remove(currentDestination);
                        sleep(100);
                        stage = Stage.PICK;
                    }
                    break;

                    case DONE:
                        gvh.plat.moat.motion_stop();
                        return null;
                }
            }
            else{
                currentDestination = getRandomElement(destinations.get(currentSet));
                gvh.plat.moat.goTo(currentDestination, obsList);
            }
            sleep(100);
        }
    }

    @Override
    protected void receive(RobotMessage m) {
        String posName = m.getContents(0);
        System.out.println("message receive test");
        if(destinations.get(currentSet).containsKey(posName))
            destinations.get(currentSet).remove(posName);

        // if no more waypoints in set, go to wait, which will go to elect
        /*if(destinations.get(currentSet).isEmpty()) {
            stage = Stage.WAIT;
            return;
        }*/

       String fromID = m.getFrom().replaceFirst("[^0-9]+", "");
       int fromInt = Integer.parseInt(fromID);
       // if i'm next robot, or the last robot and there are more points, go to pick
       if(gvh.id.getIdNumber() == fromInt + 1) {


           stage = Stage.PICK;
       }
     /*  else {
           stage = Stage.WAIT;
       }*/

       /* if(currentDestination.getName().equals(posName)) {
            gvh.plat.moat.cancel();
            stage = Stage.PICK;
        }*/

    }

    private static final Random rand = new Random();

    @SuppressWarnings("unchecked")
    private <X, T> T getRandomElement(Map<X, T> map) {
        if(RANDOM_DESTINATION)
            return (T) map.values().toArray()[rand.nextInt(map.size())];
        else
            return (T) map.values().toArray()[0];
    }
}