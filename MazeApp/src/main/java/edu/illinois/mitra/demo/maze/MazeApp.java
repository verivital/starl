package edu.illinois.mitra.demo.maze;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Stack;





import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.*;
import edu.illinois.mitra.starl.motion.*;
import edu.illinois.mitra.starl.motion.MotionParameters.COLAVOID_MODE_TYPE;

//TODO: Fix pathstack lower and uppper x/y bounds. Fix obstacles

public class MazeApp extends LogicThread {
	private static final boolean RANDOM_DESTINATION = false;
	public static final int ARRIVED_MSG = 22;
	private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
	private volatile MotionParameters param = DEFAULT_PARAMETERS;
	final Map<String, ItemPosition> destinations = new HashMap<String, ItemPosition>();
	int robotIndex;
	
	// used to find path through obstacles
	Stack<ItemPosition> pathStack;	
	RRTNode kdTree = new RRTNode();
	
	ObstacleList obsList;
	//obsList is a local map each robot has, when path planning, use this map
	ObstacleList obEnvironment;
	//obEnvironment is the physical environment, used when calculating collisions
	
	ItemPosition currentDestination, preDestination;
	
	private LeaderElection le;
//	private String leader = null;
	private boolean iamleader = false;
	
	private enum Stage {
		PICK, GO, DONE, ELECT, HOLD, MIDWAY
	};

	private Stage stage = Stage.PICK;

	public MazeApp(GlobalVarHolder gvh) {
		super(gvh);
		for(int i = 0; i< gvh.gps.get_robot_Positions().getNumPositions(); i++){
			if(gvh.gps.get_robot_Positions().getList().get(i).name == name){
				robotIndex = i;
				break;
			}
		}
		
		le = new RandomLeaderElection(gvh);
		
		
		MotionParameters.Builder settings = new MotionParameters.Builder();
//		settings.ROBOT_RADIUS(400);
		settings.COLAVOID_MODE(COLAVOID_MODE_TYPE.USE_COLBACK);
		MotionParameters param = settings.build();
		gvh.plat.moat.setParameters(param);
		
		for(ItemPosition i : gvh.gps.getWaypointPositions())
			destinations.put(i.getName(), i);
		

		//point the environment to internal data, so that we can update it 
		obEnvironment = gvh.gps.getObspointPositions();
		
		//download from environment here so that all the robots have their own copy of visible ObstacleList
		obsList = gvh.gps.getViews().elementAt(robotIndex) ;
		
		gvh.comms.addMsgListener(this, ARRIVED_MSG);
	}

	@Override
	public List<Object> callStarL() {
		while(true) {
			obEnvironment.updateObs();

			obsList.updateObs();
			if((((Model_Ground)gvh.plat.model).type == Model_Ground.Type.GET_TO_GOAL)
					|| (((Model_Ground)gvh.plat.model).type == Model_Ground.Type.EXPLORE_AREA)){
					
				switch(stage) {
				case ELECT:
					/*
					le.elect();
					if(le.getLeader() != null) {
						results[1] = le.getLeader();
					}
					*/
					stage = Stage.PICK;
	
					break;
				case PICK:
					if(destinations.isEmpty()) {
						stage = Stage.DONE;
					} else 
					{
	
			//			RobotMessage informleader = new RobotMessage("ALL", name, 21, le.getLeader());
			//			gvh.comms.addOutgoingMessage(informleader);
	
			//			iamleader = le.getLeader().equals(name);
						iamleader = true;
						
						if(iamleader)
						{
							currentDestination = getRandomElement(destinations);
							
							RRTNode path = new RRTNode(gvh.gps.getPosition(name).getX(), gvh.gps.getPosition(name).getY());

							//TODO: Fix pathstack lower and uppper x/y bounds
							pathStack = path.findRoute(currentDestination, 5000, obsList,0, 5000,0, 3000, (gvh.gps.getPosition(name)), (int) (gvh.plat.model.radius()*0.8));

							kdTree = RRTNode.stopNode;
							//wait when can not find path
							if(pathStack == null){
							stage = Stage.HOLD;	
							}					
							else{
								preDestination = null;
								stage = Stage.MIDWAY;
							}
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
					}
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
									stage = Stage.PICK;
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
								stage = Stage.PICK;
							}
							else{
								if(currentDestination != null){
									destinations.remove(currentDestination.getName());
								RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, currentDestination.getName());
								gvh.comms.addOutgoingMessage(inform);
								stage = Stage.PICK;
								}
							}
						}
					}
					break;	
					
				case GO:
					if(!gvh.plat.moat.inMotion) {
						if(currentDestination != null)
							destinations.remove(currentDestination.getName());
						RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, currentDestination.getName());
						gvh.comms.addOutgoingMessage(inform);
						stage = Stage.PICK;
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
					}
					break;

				case DONE:
					gvh.plat.moat.motion_stop();
					return null;
				}
			}
			else{
				currentDestination = getRandomElement(destinations);
				gvh.plat.moat.goTo(currentDestination, obsList);
			}
			sleep(100);
		}
	}

	@Override
	protected void receive(RobotMessage m) {
		String posName = m.getContents(0);
		if(destinations.containsKey(posName))
			destinations.remove(posName);

		if(currentDestination.getName().equals(posName)) {
			gvh.plat.moat.cancel();
			stage = Stage.PICK;
		}
		
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