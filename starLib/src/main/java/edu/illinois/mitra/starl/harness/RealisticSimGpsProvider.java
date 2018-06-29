package edu.illinois.mitra.starl.harness;

import java.util.HashMap;
import java.util.Map;
import java.util.Observable;
import java.util.Observer;
import java.util.Vector;
import java.util.concurrent.ConcurrentHashMap;

import edu.illinois.mitra.starl.interfaces.TrackedRobot;
import edu.illinois.mitra.starl.models.Model_3DR;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starl.models.Model_Mavic;
import edu.illinois.mitra.starl.models.Model_Phantom;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Obstacles;
import edu.illinois.mitra.starl.objects.Point3d;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * This defines methods for initializing motion environment related objects  
 * It also updates robots' movement, collision, other sensor info
 *
 * @author Yixiao Lin & Adam Zimmerman
 * @version 2.0
 */

public class RealisticSimGpsProvider extends Observable implements SimGpsProvider {	
	private Map<String, SimGpsReceiver> receivers;
	private Map<String, TrackedModel<Model_iRobot>> iRobots;
	private Map<String, TrackedModel<Model_quadcopter>> quadcopters;
	private Map<String, TrackedModel<Model_GhostAerial>> ghosts;
	private Map<String, TrackedModel<Model_Mavic>> mavics;
	private Map<String, TrackedModel<Model_3DR>> o3DRs;
	private Map<String, TrackedModel<Model_Phantom>> phantoms;
	

	// Waypoint positions and robot positions that are shared among all robots	
	private PositionList<Model_iRobot> iRobot_positions;
	private PositionList<Model_quadcopter> quadcopter_positions;
	private PositionList<Model_GhostAerial> ghost_positions;
	private PositionList<Model_Mavic> mavic_positions;
	private PositionList<Model_Phantom> phantom_positions;
	private PositionList<Model_3DR> o3DR_positions;
	private PositionList<ItemPosition> allpos;

	private PositionList<ItemPosition> waypoint_positions;
	private PositionList<ItemPosition> sensepoint_positions;
	private ObstacleList obspoint_positions;
	private Vector<ObstacleList> viewsOfWorld;
	
	private long period = 100;
	private double[] noises;

	private SimulationEngine se;
		
	public RealisticSimGpsProvider(SimulationEngine se, long period, double angleNoise, double posNoise) {
		this.se = se;
		this.period = period;
		//TODO: get noise from sim settings or motion parameter, need to get a generalized version of noise
		noises = new double[3];
		noises[0] = posNoise;
		noises[1] = posNoise;
		noises[2] = angleNoise;
		
		receivers = new HashMap<String, SimGpsReceiver>();
		iRobots = new ConcurrentHashMap<String, TrackedModel<Model_iRobot>>();
		quadcopters = new ConcurrentHashMap<String, TrackedModel<Model_quadcopter>>();
		ghosts = new ConcurrentHashMap<String, TrackedModel<Model_GhostAerial>>();
		mavics = new ConcurrentHashMap<String, TrackedModel<Model_Mavic>>();
		phantoms = new ConcurrentHashMap<String, TrackedModel<Model_Phantom>>();
		o3DRs = new ConcurrentHashMap<String, TrackedModel<Model_3DR>>();

		waypoint_positions = new PositionList<ItemPosition>();
		sensepoint_positions = new PositionList<ItemPosition>();
		iRobot_positions = new PositionList<Model_iRobot>();
		allpos = new PositionList<ItemPosition>();
		quadcopter_positions = new PositionList<Model_quadcopter>();
		ghost_positions = new PositionList<Model_GhostAerial>();
		mavic_positions = new PositionList<Model_Mavic>();
		phantom_positions = new PositionList<Model_Phantom>();
		o3DR_positions = new PositionList<Model_3DR>();
	}
	
	@Override
	public synchronized void registerReceiver(String name, SimGpsReceiver simGpsReceiver) {
		receivers.put(name, simGpsReceiver);
	}
	
	@Override
	public synchronized void addRobot(TrackedRobot bot) {
		allpos.update((ItemPosition)bot);

		if(bot instanceof Model_iRobot){
			synchronized(iRobots) {
				iRobots.put(((Model_iRobot)bot).name, new TrackedModel<Model_iRobot>((Model_iRobot) bot));
			}
			iRobot_positions.update((Model_iRobot) bot);
			
		}
		else if(bot instanceof Model_quadcopter){
			synchronized(quadcopters) {
				quadcopters.put(((Model_quadcopter)bot).name, new TrackedModel<Model_quadcopter>((Model_quadcopter) bot));
			}
			quadcopter_positions.update((Model_quadcopter) bot);
		}else if(bot instanceof Model_GhostAerial){
			synchronized(ghosts) {
				ghosts.put(((Model_GhostAerial)bot).name, new TrackedModel<Model_GhostAerial>((Model_GhostAerial) bot));
			}
			ghost_positions.update((Model_GhostAerial) bot);
		}else if(bot instanceof Model_Mavic) {
			synchronized (mavics) {
				mavics.put(((Model_Mavic) bot).name, new TrackedModel<Model_Mavic>((Model_Mavic) bot));
			}
			mavic_positions.update((Model_Mavic) bot);
		}else if(bot instanceof Model_Phantom){
			synchronized(phantoms){
				phantoms.put(((Model_Phantom) bot).name, new TrackedModel<Model_Phantom>((Model_Phantom) bot));
			}
		}else if(bot instanceof Model_3DR){
			synchronized(o3DRs) {
				o3DRs.put(((Model_3DR)bot).name, new TrackedModel<Model_3DR>((Model_3DR) bot));
			}
			o3DR_positions.update((Model_3DR) bot);
		} 
		else{
			throw new RuntimeException("after adding a new model, one need to add model handling in simulation under RealisticSimGpsProvider");
		}
		
	}
	
	@Override
	public synchronized void setDestination(String name, ItemPosition dest, int vel) {
		throw new RuntimeException("setDestination is not implemented for realistic simulated motion! " +
				"RealisticSimGpsProvider MUST be used with RealisticSimMotionAutomaton");
	}

	@Override
	public void setVelocity(String name, int fwd, int rad) {
		((Model_iRobot) iRobots.get(name).cur).vFwd = fwd;
		((Model_iRobot) iRobots.get(name).cur).vRad = rad;
	}
	
	@Override
	public void setControlInput(String name, double v_yaw, double pitch, double roll, double gaz) {
		/** TODO: replace with PID model here
		*/

		((Model_quadcopter) quadcopters.get(name).cur).v_yawR = v_yaw;
		((Model_quadcopter) quadcopters.get(name).cur).pitchR = pitch;
		((Model_quadcopter) quadcopters.get(name).cur).rollR = roll;	
		((Model_quadcopter) quadcopters.get(name).cur).gazR = gaz;	
	}

	//Called from realisticSimMotionAutomaton_ghost
	@Override
	public void setControlInputGA(String name, double v_yaw, double pitch, double roll, double gaz) {
		/** TODO: replace with PID model here
		 */
		((Model_GhostAerial) ghosts.get(name).cur).v_yawR = v_yaw;
		((Model_GhostAerial) ghosts.get(name).cur).pitchR = pitch;
		((Model_GhostAerial) ghosts.get(name).cur).rollR = roll;
		((Model_GhostAerial) ghosts.get(name).cur).gazR = gaz;
	}

	//Called from realisticSimMotionAutomaton_mavic
	@Override
	public void setControlInputMav(String name, double v_yaw, double pitch, double roll, double gaz) {
		/** TODO: replace with PID model here
		 */
		((Model_Mavic) mavics.get(name).cur).v_yawR = v_yaw;
		((Model_Mavic) mavics.get(name).cur).pitchR = pitch;
		((Model_Mavic) mavics.get(name).cur).rollR = roll;
		((Model_Mavic) mavics.get(name).cur).gazR = gaz;
	}

	//Called from realisticSimMotionAutomaton_phantom
	@Override
	public void setControlInputPhantom(String name, double v_yaw, double pitch, double roll, double gaz) {
		/** TODO: replace with PID model here
		 */
		((Model_Phantom) phantoms.get(name).cur).v_yawR = v_yaw;
		((Model_Phantom) phantoms.get(name).cur).pitchR = pitch;
		((Model_Phantom) phantoms.get(name).cur).rollR = roll;
		((Model_Phantom) phantoms.get(name).cur).gazR = gaz;
	}

	//Called from realisticSimMotionAutomaton_3DR
	//@Override
	public void setControlInput3DR(String name, double v_yaw, double pitch, double roll, double gaz) {
		/** TODO: replace with PID model here
		 */
		((Model_3DR) o3DRs.get(name).cur).v_yawR = v_yaw;
		((Model_3DR) o3DRs.get(name).cur).pitchR = pitch;
		((Model_3DR) o3DRs.get(name).cur).rollR = roll;
		((Model_3DR) o3DRs.get(name).cur).gazR = gaz;
	}

	@Override
	public synchronized void halt(String name) {
		setVelocity(name, 0, 0);
		setControlInput(name, 0, 0, 0, 0);
	}
	
	@Override
	public PositionList<Model_iRobot> getiRobotPositions() {
		return iRobot_positions;
	}
	
	@Override
	public PositionList<Model_quadcopter> getQuadcopterPositions() {
		return quadcopter_positions;
	}

	@Override
	public PositionList<Model_GhostAerial> getGhostAerialsPositions() {
		return ghost_positions;
	}

	@Override
	public PositionList<Model_Mavic> getMavicPositions() {
		return mavic_positions;
	}
	
	@Override
	public PositionList<Model_Phantom> getPhantomPositions() {return phantom_positions;}

	@Override
	public PositionList<Model_3DR> get3DRPositions() {
		return o3DR_positions;
	}

	@Override
	public PositionList<ItemPosition> getAllPositions(){
		return allpos;
	}

	@Override
	public void setWaypoints(PositionList<ItemPosition> loadedWaypoints) {
		if(loadedWaypoints != null) waypoint_positions = loadedWaypoints;
	}
	
	@Override
	public void setSensepoints(PositionList<ItemPosition> loadedSensepoints) {
		if(loadedSensepoints != null) sensepoint_positions = loadedSensepoints;
	}
	
	@Override
	public void setObspoints(ObstacleList loadedObspoints) {
		if(loadedObspoints != null) obspoint_positions = loadedObspoints;
	}
	
	
	@Override
	public void setViews(ObstacleList environment, int nBots) {
		if(environment != null){
			viewsOfWorld = new Vector<ObstacleList>(3,2);
			ObstacleList obsList = null;
			for(int i = 0; i< nBots ; i++){
				obsList = environment.downloadObs();
				obsList.Gridfy();
				viewsOfWorld.add(obsList);
			}
		}
	}
	

	@Override
	public ObstacleList getObspointPositions() {
		return obspoint_positions;
	}
	
	@Override
	public PositionList<ItemPosition> getWaypointPositions() {
		return waypoint_positions;
	}
	
	@Override
	public PositionList<ItemPosition> getSensePositions() {
		return sensepoint_positions;
	}
	
	@Override
	public Vector<ObstacleList> getViews() {
		return viewsOfWorld;
	}
	
	@Override
	public void start() {
		// Create a periodic runnable which repeats every "period" ms to report positions
		Thread posupdate = new Thread() {
			@Override
			public void run() {
				Thread.currentThread().setName("RealisticGpsProvider");
				se.registerThread(this);
				
				while(true) {
					synchronized(iRobots) {
						for(TrackedModel<Model_iRobot> r : iRobots.values()) {
							//if(r.cur.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition((r.cur.inMotion()));	
							//}
						}	
					}
					synchronized(quadcopters) {
						for(TrackedModel<Model_quadcopter> r : quadcopters.values()) {
							//if(r.cur.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition((r.cur.inMotion()));	
							//}
						}	
					}
					synchronized(ghosts) {
						for(TrackedModel<Model_GhostAerial> r : ghosts.values()) {
							//if(r.cur.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition((r.cur.inMotion()));
							//}
						}
					}

					synchronized(mavics) {
						for(TrackedModel<Model_Mavic> r : mavics.values()) {
							//if(r.cur.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition((r.cur.inMotion()));
							//}
						}
					}
					synchronized(phantoms) {
						for(TrackedModel<Model_Phantom> r : phantoms.values()) {
							//if(r.cur.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition((r.cur.inMotion()));
							//}
						}
					}
					synchronized(o3DRs) {
						for(TrackedModel<Model_3DR> r : o3DRs.values()) {
							//if(r.cur.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition((r.cur.inMotion()));
							//}
						}
					}
					setChanged();
					notifyObservers(allpos);
				//	notifyObservers(quadcopter_positions);
					
					try {
						se.threadSleep(period, this);
						Thread.sleep(Long.MAX_VALUE);
					} catch (InterruptedException e) {
					}
				}
			}
		};
		posupdate.start();
	}	
	
	
	@Override
	public void notifyObservers(Object data) {
		// Catch NullPointerExceptions by ignorning null data
		if(data != null) super.notifyObservers(data);
	}

	private class TrackedModel<T extends ItemPosition & TrackedRobot>{
		//private boolean stopMoving = false;
		private T cur = null;
		private long timeLastUpdate = 0;
		
		public TrackedModel(T pos) {
			this.cur = pos;
			timeLastUpdate = se.getTime();
			pos.initialize();
		}
		public void updatePos() {
			double timeSinceUpdate = (se.getTime() - timeLastUpdate)/1000.0;
			
			Point3d p_point = cur.predict(noises, timeSinceUpdate);
			boolean collided = checkCollision(p_point);
			//boolean collided = false; todo(tim) address collisions
			cur.updatePos(!collided);
			
			cur.updateSensor(obspoint_positions, sensepoint_positions);

			timeLastUpdate = se.getTime();
		}
		
		public boolean checkCollision(Point3d bot) {
			//double min_distance = Double.MAX_VALUE;
			int myRadius = 0;
			if(cur instanceof Model_iRobot){
				myRadius = ((Model_iRobot) cur).radius;
			}
			else if(cur instanceof Model_quadcopter){
				myRadius = ((Model_quadcopter) cur).radius;
			}else if(cur instanceof Model_GhostAerial){
				myRadius = ((Model_GhostAerial) cur).radius;
			}else if(cur instanceof Model_Mavic) {
				myRadius = ((Model_Mavic) cur).radius;
			} else if(cur instanceof Model_Phantom){
				myRadius = ((Model_Phantom) cur).radius;
			}else if(cur instanceof Model_3DR){
				myRadius = ((Model_3DR) cur).radius;
			}
			else{
				throw new RuntimeException("after adding a new model, one need to add collision handling under RealisticSimGpsProvider");
			}
			
			boolean toReturn = false;
			
			for(Model_iRobot current : iRobot_positions.getList()) {
				if(!current.name.equals(cur.name)) {
					if(bot.distanceTo(current) <= myRadius + current.radius){
						//update sensors for both robots
						current.collision(cur);
						cur.collision(current);
						toReturn = true;
					}
					//min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
				}
			}
			
			for(Model_quadcopter current : quadcopter_positions.getList()) {
				if(!current.name.equals(cur.name)) {
					if(bot.distanceTo(current) <= myRadius + current.radius){
						//update sensors for both robots
						current.collision(cur);
						cur.collision(current);
						toReturn = true;
					}
					//min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
				}
			}

			for(Model_GhostAerial current : ghost_positions.getList()) {
				if(!current.name.equals(cur.name)) {
					if(bot.distanceTo(current) <= myRadius + current.radius){
						//update sensors for both robots
						current.collision(cur);
						cur.collision(current);
						toReturn = true;
					}
					//min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
				}
			}

			for(Model_Mavic current : mavic_positions.getList()) {
				if(!current.name.equals(cur.name)) {
					if(bot.distanceTo(current) <= myRadius + current.radius){
						//update sensors for both robots
						current.collision(cur);
						cur.collision(current);
						toReturn = true;
					}
					//min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
				}
			}
			for(Model_Phantom current : phantom_positions.getList()) {
				if(!current.name.equals(cur.name)) {
					if(bot.distanceTo(current) <= myRadius + current.radius){
						//update sensors for both robots
						current.collision(cur);
						cur.collision(current);
						toReturn = true;
					}
					//min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
				}
			}
			for(Model_3DR current : o3DR_positions.getList()) {
				if(!current.name.equals(cur.name)) {
					if(bot.distanceTo(current) <= myRadius + current.radius){
						//update sensors for both robots
						current.collision(cur);
						cur.collision(current);
						toReturn = true;
					}
					//min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
				}
			}
			
			ObstacleList list = obspoint_positions;
			for(int i = 0; i < list.ObList.size(); i++)
			{
				Obstacles currobs = list.ObList.get(i);
				Point3d nextpoint = currobs.obstacle.firstElement();
				Point3d curpoint = currobs.obstacle.firstElement();
				ItemPosition wall = new ItemPosition("wall",0,0,0);
				
				for(int j = 0; j < currobs.obstacle.size() ; j++){
					curpoint = currobs.obstacle.get(j);
					if (j == currobs.obstacle.size() -1){
						nextpoint = currobs.obstacle.firstElement();
					}
					else{
						nextpoint = currobs.obstacle.get(j+1);
					}
					Point3d closeP = currobs.getClosestPointOnSegment(curpoint.x(), curpoint.y(), nextpoint.x(), nextpoint.y(), bot.x(), bot.y());
					wall.set(closeP.x(), closeP.y(), 0);
					double distance = Math.sqrt(Math.pow(closeP.x() - bot.x(), 2) + Math.pow(closeP.y() - bot.y(), 2)) ;
					
					//need to modify some conditions of bump sensors, we have left and right bump sensor for now
					if(distance < myRadius){
						//update the bump sensor 
						cur.collision(wall);
						toReturn = true;
					}
				}
			}
			if(!toReturn){
				cur.collision(null);
			}
			return toReturn;
		}
		
		public String getName() {
			return cur.name;
		}
		
	}
	
	@Override
	public void addObserver(Observer o) {
		super.addObserver(o);
	}
	
}
