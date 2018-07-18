package edu.illinois.mitra.demo.search;

import java.util.*;

import edu.illinois.mitra.starl.comms.MessageContents;
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.*;
import edu.illinois.mitra.starl.motion.*;
import edu.illinois.mitra.starl.motion.MotionParameters.COLAVOID_MODE_TYPE;

public class DistributedSearchApp extends LogicThread {
	ItemPosition CS_A = new ItemPosition("CS_A", 2250, 2750, 0);
	ItemPosition CS_B = new ItemPosition("CS_B", 2750, 2750, 0);
	ItemPosition CS_C = new ItemPosition("CS_C", 2250, 2250, 0);
	ItemPosition CS_D = new ItemPosition("CS_D", 2750, 2250, 0);
	public static final int TASK_MSG = 21;
	public static final int ASSIGN_MSG = 22;
	public static final int FAILED_MSG = 23;
	public static final int DONE_MSG = 24;
	public static final int FOUND_MSG = 25;
	private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
	private volatile MotionParameters param = DEFAULT_PARAMETERS;
	Queue<ItemPosition> destinations = new LinkedList<ItemPosition>();
	Queue<ItemPosition> Alldest;
	public PositionList SensePt;
	//SensePt is just for display purpose
	private LeaderElection le;
	private boolean iamleader=false;
	private long Start_time;
	
	// used to find path through obstacles
	Stack<ItemPosition> pathStack;	
	RRTNode kdTree = new RRTNode();
	
	LinkedList<ItemPosition> searchPath;
	
	ObstacleList obEnvironment;
	int robotIndex;
	ItemPosition currentDestination, preDestination, searchTemp ;
		
	private enum Stage {
		ELECT, ASSIGN, PICK, GO, HOLD, SEARCH, DONE
	};

	private Stage stage = Stage.ELECT;

	public DistributedSearchApp(GlobalVarHolder gvh) {
		super(gvh);
		
		robotIndex = Integer.parseInt(name.substring(6,name.length()));
		MotionParameters.Builder settings = new MotionParameters.Builder();
		settings.COLAVOID_MODE(COLAVOID_MODE_TYPE.USE_COLBACK);
		//settings.GOAL_RADIUS(15);
		param = settings.build();
		gvh.plat.moat.setParameters(param);
		obEnvironment = gvh.gps.getObspointPositions();

		le = new RandomLeaderElection(gvh);
		gvh.comms.addMsgListener(this, TASK_MSG);
		gvh.comms.addMsgListener(this, ASSIGN_MSG);
		gvh.comms.addMsgListener(this, DONE_MSG);
		gvh.comms.addMsgListener(this, FAILED_MSG);
		gvh.comms.addMsgListener(this, FOUND_MSG);
		
		
	}

	@Override
	public List<Object> callStarL() {
		int itr = 0;
		while(true) {
			if(gvh.gps.getMyPosition().circleSensor && stage != Stage.DONE){
				MessageContents content = new MessageContents("Got it!");
				RobotMessage got_it_msg = new RobotMessage("ALL", name, FOUND_MSG, content);
				gvh.comms.addOutgoingMessage(got_it_msg);
				stage= Stage.DONE;
				System.out.println(name + " Got it!");
			}
			if((((Model_Ground)gvh.plat.model).type == Model_Ground.Type.GET_TO_GOAL)
						|| (((Model_Ground)gvh.plat.model).type == Model_Ground.Type.EXPLORE_AREA)){

				switch(stage) {
				case ELECT:
					if(itr == 0){
						le.elect();
					}
					if(le.getLeader() != null) {
						iamleader = le.getLeader().equals(name);
						if(iamleader){
							System.out.println(name);
						}
						stage = Stage.ASSIGN;
					}
					break;
				case ASSIGN:
					if(iamleader){
						int size = gvh.gps.getWaypointPositions().getNumPositions();
						int fleet = gvh.gps.get_robot_Positions().getNumPositions();
						int mul = (Integer)size/fleet;
						int remain = size%fleet;
						int index = 0;
						Alldest = new LinkedList<ItemPosition>();
						for(int i=0; i < size; i+=mul){
							for(int j = 0; j < mul; j++){
								ItemPosition toAdd = new ItemPosition(gvh.gps.getWaypointPositions().getList().get(i+j));
								Alldest.add(toAdd);
								assign(toAdd, index);
							}
							if(remain>0){
								ItemPosition toAdd = new ItemPosition(gvh.gps.getWaypointPositions().getList().get(i+mul));
								Alldest.add(toAdd);
								remain--;
								i++;
								assign(toAdd, index);
							}
							index ++;
						}
						//assign finished, tell individual to start
						MessageContents content = new MessageContents("20000");
						RobotMessage start_task_msg = new RobotMessage("ALL", name, TASK_MSG, content);
						gvh.comms.addOutgoingMessage(start_task_msg);
						Start_time = 20000*robotIndex + gvh.time();
						stage = Stage.PICK;
					}
					break;	
				case PICK:
					if(gvh.time()>Start_time){
						if(destinations.isEmpty()) {
							RobotMessage done_task_msg = new RobotMessage("ALL", name, DONE_MSG, "done");
							gvh.comms.addOutgoingMessage(done_task_msg);
							stage = Stage.DONE;
						} else 
						{
							currentDestination = (ItemPosition)destinations.peek();
							RRTNode path = new RRTNode(gvh.gps.getPosition(name).getX(), gvh.gps.getPosition(name).getY());

							//TODO: Fix x/y upper and lower limits
							pathStack = path.findRoute(currentDestination, 5000, obEnvironment, 12330, 0,0, 8500, (gvh.gps.getPosition(name)), (int) (gvh.plat.model.radius()));
							kdTree = RRTNode.stopNode;
							//wait when can not find path
							if(pathStack == null){
								stage = Stage.HOLD;	
							}					
							else{
								preDestination = null;
								stage = Stage.GO;
							}
						}
						break;
					}
				case HOLD:
					break;
				case GO:
					if(!gvh.plat.moat.inMotion) {
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
							gvh.plat.moat.goTo(goMidPoint);
						}
						else{
							if((gvh.gps.getPosition(name).distanceTo(currentDestination)>param.GOAL_RADIUS)){
								pathStack.clear();
								stage = Stage.PICK;
							}
							else{
								//reached the point, go on searching
								searchPath = robotCoverageAlg(currentDestination);
								searchTemp = currentDestination;
								stage = Stage.SEARCH;
							}
						}
					}
					break;
				case SEARCH:
					
					if(!gvh.plat.moat.inMotion) {
						if(searchTemp.distanceTo(gvh.gps.getMyPosition())>100){
							MessageContents content = new MessageContents(currentDestination.name);
							RobotMessage task_fail_msg = new RobotMessage("ALL", name, FAILED_MSG, content);
							gvh.comms.addOutgoingMessage(task_fail_msg);
							stage= Stage.DONE;
							break;
						}
						if(searchPath.isEmpty()){
							if(currentDestination != null){
								destinations.remove();
								//finished searching this room, go to the next one
								stage = Stage.PICK;
								//System.out.println("reached here");
							}
							else{
								gvh.log.e("Error from state machine", "should not reach here if in SEARCH stage");
							}
						}
						else{
							gvh.plat.moat.goTo(searchPath.peek());
							searchTemp = searchPath.poll();
						}
					}
					break;
				case DONE:
					gvh.plat.moat.motion_stop();
				
					//if does not return null, program will never halt
					//useful for debugging
					//return null;
					break;
				}
				itr ++;
			}
			sleep(100);
		}
	}


	private void assign(ItemPosition toAdd, int index) {
		//send the assign message to indexed bot
		String botS = "iRobot"+index;
		if(botS.equals(name)){
			destinations.add(toAdd);
			return;
		}
		String[] temp = new String[3];
		temp[0] = toAdd.name;
		temp[1] = Integer.toString(toAdd.getX());
		temp[2] = Integer.toString(toAdd.getY());
		MessageContents content = new MessageContents(temp);
		RobotMessage assign_msg = new RobotMessage(botS, name, ASSIGN_MSG, content);
		gvh.comms.addOutgoingMessage(assign_msg);
		System.out.println("Assign "+ content+ " to "+botS);
	}

	@Override
	protected void receive(RobotMessage m) {
		if(m.getTo().equals(name) || m.getTo().equals("ALL")){
			if(m.getMID() == ASSIGN_MSG){
			//	System.out.println("bot "+name + " got point " + m.getContents(0));
				MessageContents msg_content = m.getContents();
				List<String> assignedP = new ArrayList<String>(msg_content.getContents());
				ItemPosition point = new ItemPosition(assignedP.get(0), Integer.parseInt(assignedP.get(1)), Integer.parseInt(assignedP.get(2)), 0);
				destinations.add(point);
			}
			if(m.getMID() == TASK_MSG){
				//	System.out.println("bot "+name + " got point " + m.getContents(0));
					MessageContents msg_content = m.getContents();
					Start_time = Integer.parseInt(msg_content.getContents().get(0))*robotIndex + gvh.time();
					stage = Stage.PICK;
			}
			if(m.getMID() == FOUND_MSG){
				//	System.out.println("bot "+name + " got point " + m.getContents(0));
					stage = Stage.DONE;
			}
		}
	}
	
	private LinkedList<ItemPosition> robotCoverageAlg(ItemPosition door){
		//This is the algorithm for finding a path that will cover the room. We will not focus on this algorithm, therefore we pre-enter the points
		LinkedList<ItemPosition> toReturn = new LinkedList<ItemPosition>();
		switch(door.name){
			case "A":
				toReturn.add(new ItemPosition("temp",2800,3000,0));
				toReturn.add(new ItemPosition("temp",3400,3000,0));
				toReturn.add(new ItemPosition("temp",3400,320,0));
				toReturn.add(new ItemPosition("temp",3000,320,0));
				toReturn.add(new ItemPosition("temp",3000,3000,0));
				toReturn.add(new ItemPosition("temp",3000,320,0));
				toReturn.add(new ItemPosition("temp",2600,320,0));
				toReturn.add(new ItemPosition("temp",2600,3000,0));
				toReturn.add(new ItemPosition("temp",2200,3000,0));
				toReturn.add(new ItemPosition("temp",2200,320,0));
				toReturn.add(new ItemPosition("temp",1800,320,0));
				toReturn.add(new ItemPosition("temp",1800,4000,0));
				toReturn.add(new ItemPosition("temp",1400,4000,0));
				toReturn.add(new ItemPosition("temp",1400,320,0));
				toReturn.add(new ItemPosition("temp",1000,320,0));
				toReturn.add(new ItemPosition("temp",1000,4000,0));
				toReturn.add(new ItemPosition("temp",400,4000,0));
				toReturn.add(new ItemPosition("temp",400,320,0));
				break;
			case "B":
				toReturn.add(new ItemPosition("temp",3000,6200,0));
				toReturn.add(new ItemPosition("temp",2600,6200,0));
				toReturn.add(new ItemPosition("temp",2600,4500,0));
				toReturn.add(new ItemPosition("temp",2200,4500,0));
				toReturn.add(new ItemPosition("temp",2200,6200,0));
				toReturn.add(new ItemPosition("temp",1800,6200,0));
				toReturn.add(new ItemPosition("temp",1800,4500,0));
				toReturn.add(new ItemPosition("temp",1400,4500,0));
				toReturn.add(new ItemPosition("temp",1400,6200,0));
				toReturn.add(new ItemPosition("temp",1000,6200,0));
				toReturn.add(new ItemPosition("temp",1000,4500,0));
				toReturn.add(new ItemPosition("temp",600,4500,0));
				toReturn.add(new ItemPosition("temp",600,6200,0));
				toReturn.add(new ItemPosition("temp",350,6200,0));
				toReturn.add(new ItemPosition("temp",350,4500,0));
				break;
			case "C":
				toReturn.add(new ItemPosition("temp",5200,2800,0));
				toReturn.add(new ItemPosition("temp",7600,3000,0));
				toReturn.add(new ItemPosition("temp",7600,320,0));
				toReturn.add(new ItemPosition("temp",7200,320,0));
				toReturn.add(new ItemPosition("temp",7200,3000,0));
				toReturn.add(new ItemPosition("temp",6800,3000,0));
				toReturn.add(new ItemPosition("temp",6800,320,0));
				toReturn.add(new ItemPosition("temp",6400,320,0));			
				toReturn.add(new ItemPosition("temp",6400,3000,0));
				toReturn.add(new ItemPosition("temp",6000,3000,0));
				toReturn.add(new ItemPosition("temp",6000,320,0));
				toReturn.add(new ItemPosition("temp",5600,320,0));
				toReturn.add(new ItemPosition("temp",5600,3000,0));
				toReturn.add(new ItemPosition("temp",5200,3000,0));
				toReturn.add(new ItemPosition("temp",5200,320,0));
				toReturn.add(new ItemPosition("temp",4800,320,0));
				toReturn.add(new ItemPosition("temp",4800,3000,0));
				toReturn.add(new ItemPosition("temp",4400,3000,0));
				toReturn.add(new ItemPosition("temp",4400,320,0));
				toReturn.add(new ItemPosition("temp",4000,320,0));
				toReturn.add(new ItemPosition("temp",4000,3000,0));
				break;
			case "D":
				toReturn.add(new ItemPosition("temp",6400,4800,0));
				toReturn.add(new ItemPosition("temp",3600,4500,0));
				toReturn.add(new ItemPosition("temp",3600,4900,0));
				toReturn.add(new ItemPosition("temp",6400,4900,0));
				toReturn.add(new ItemPosition("temp",6400,5300,0));
				toReturn.add(new ItemPosition("temp",3600,5300,0));
				toReturn.add(new ItemPosition("temp",3600,5700,0));
				toReturn.add(new ItemPosition("temp",6400,5700,0));			
				toReturn.add(new ItemPosition("temp",6400,6100,0));
				toReturn.add(new ItemPosition("temp",3600,6100,0));
				toReturn.add(new ItemPosition("temp",3600,6500,0));
				toReturn.add(new ItemPosition("temp",6400,6500,0));
				toReturn.add(new ItemPosition("temp",6400,6900,0));
				toReturn.add(new ItemPosition("temp",3600,6900,0));
				toReturn.add(new ItemPosition("temp",3600,7300,0));
				toReturn.add(new ItemPosition("temp",6400,7300,0));
				toReturn.add(new ItemPosition("temp",6400,7700,0));
				toReturn.add(new ItemPosition("temp",3600,7700,0));
				toReturn.add(new ItemPosition("temp",3600,8100,0));
				toReturn.add(new ItemPosition("temp",6400,8100,0));
				toReturn.add(new ItemPosition("temp",8300,8100,0));
				toReturn.add(new ItemPosition("temp",8300,7000,0));
				toReturn.add(new ItemPosition("temp",7900,7000,0));
				toReturn.add(new ItemPosition("temp",7900,8100,0));
				toReturn.add(new ItemPosition("temp",7500,8100,0));
				toReturn.add(new ItemPosition("temp",7500,7000,0));
				toReturn.add(new ItemPosition("temp",7100,7000,0));
				toReturn.add(new ItemPosition("temp",7100,8100,0));
				break;
			case "E":
				toReturn.add(new ItemPosition("temp",7100,6300,0));
				toReturn.add(new ItemPosition("temp",7500,6300,0));
				toReturn.add(new ItemPosition("temp",7500,4600,0));
				toReturn.add(new ItemPosition("temp",7900,4600,0));
				toReturn.add(new ItemPosition("temp",7900,6300,0));
				toReturn.add(new ItemPosition("temp",8300,6300,0));
				toReturn.add(new ItemPosition("temp",8300,4600,0));
				break;
			case "F":
				toReturn.add(new ItemPosition("temp",11800,3000,0));
				toReturn.add(new ItemPosition("temp",11800,1000,0));
				toReturn.add(new ItemPosition("temp",11400,1000,0));
				toReturn.add(new ItemPosition("temp",11400,3000,0));
				toReturn.add(new ItemPosition("temp",11000,3000,0));
				toReturn.add(new ItemPosition("temp",11000,1000,0));
				toReturn.add(new ItemPosition("temp",10600,1000,0));
				toReturn.add(new ItemPosition("temp",10600,2100,0));
				toReturn.add(new ItemPosition("temp",9900,2100,0));
				toReturn.add(new ItemPosition("temp",9900,1000,0));
				toReturn.add(new ItemPosition("temp",9500,1000,0));
				toReturn.add(new ItemPosition("temp",9500,2100,0));
				break;
			case "G":
				toReturn.add(new ItemPosition("temp",9200,8000,0));
				toReturn.add(new ItemPosition("temp",9600,8000,0));
				toReturn.add(new ItemPosition("temp",9600,3600,0));
				toReturn.add(new ItemPosition("temp",10000,3600,0));
				toReturn.add(new ItemPosition("temp",10000,8000,0));
				toReturn.add(new ItemPosition("temp",10400,8000,0));
				toReturn.add(new ItemPosition("temp",10400,3600,0));
				toReturn.add(new ItemPosition("temp",10800,3600,0));
				toReturn.add(new ItemPosition("temp",10800,8000,0));
				toReturn.add(new ItemPosition("temp",11200,8000,0));
				toReturn.add(new ItemPosition("temp",11200,3600,0));
				toReturn.add(new ItemPosition("temp",11600,3600,0));
				toReturn.add(new ItemPosition("temp",11600,8000,0));
				toReturn.add(new ItemPosition("temp",12000,8000,0));
				toReturn.add(new ItemPosition("temp",12000,3600,0));
				break;
		}
		return toReturn;
	}
	




}