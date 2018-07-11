package edu.illinois.mitra.starl.harness;

import java.util.HashMap;
import java.util.Map;
import java.util.Observable;
import java.util.Observer;
import java.util.Vector;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Obstacles;
import edu.illinois.mitra.starl.objects.Point3i;
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

	/*
	 * Accesses or creates a map of Models of the specified typename. Replaces previous
	 *     private Map<String, TrackedModel<Model_iRobot>> iRobots;
	 *     ...
	 *     private Map<String, TrackedModel<Model_Phantom>> phantoms;
     */
	private Map<String, TrackedModel<Model>> getModels(String typename) {
		Map<String, TrackedModel<Model>> subMap = models.get(typename);
		if (subMap == null) {
			subMap = new HashMap<>();
			models.put(typename, subMap);
		}
		return subMap;
	}
	private final Map<String, Map<String, TrackedModel<Model>>> models;


	// Represents waypoint modelPositions and robot modelPositions that are shared among all robots.

	/*
	 * Accesses or creates a PositionList of Models of the specified type.
	 */
	private PositionList<Model> getModelPositions(String typename) {
		PositionList<Model> list = modelPositions.get(typename);
		if (list == null) {
			list = new PositionList<>();
			modelPositions.put(typename, list);
		}
		return list;
	}

	private final Map<String, PositionList<Model>> modelPositions;
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

		receivers = new HashMap<>();

		models = new HashMap<>();
		modelPositions = new HashMap<>();

		allpos = new PositionList<>();
		waypoint_positions = new PositionList<>();
		sensepoint_positions = new PositionList<>();
	}

	@Override
	public synchronized void registerReceiver(String name, SimGpsReceiver simGpsReceiver) {
		receivers.put(name, simGpsReceiver);
	}

	@Override
	public synchronized void addRobot(Model bot) {
		allpos.update(bot);

		String typeName = bot.getTypeName();
		synchronized(models) {
			getModels(typeName).put(bot.name, new TrackedModel<>(bot));
		}
		getModelPositions(typeName).update(bot);
	}

	@Override
	public synchronized void setDestination(String name, ItemPosition dest, int vel) {
		throw new RuntimeException("setDestination is not implemented for realistic simulated motion! " +
				"RealisticSimGpsProvider MUST be used with RealisticSimMotionAutomaton");
	}

	@Override
	public void setVelocity(String typename, String name, int fwd, int rad) {
		Model_iRobot iRobot = (Model_iRobot)getModels(typename).get(name).cur;
		iRobot.vFwd = fwd;
		iRobot.vRad = rad;
	}

	@Override
	public void setControlInput(String typename, String name, double v_yaw, double pitch, double roll, double gaz) {
		/** TODO: replace with PID model here
		*/

        Model_Drone drone = (Model_Drone)models.get(typename).get(name).cur;
		drone.setV_yawR(v_yaw);
		drone.setPitchR(pitch);
		drone.setRollR(roll);
		drone.setGazR(gaz);
	}

    @Override
    public PositionList<ItemPosition> getAllPositions() {
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
		// Create a periodic runnable which repeats every "period" ms to report modelPositions
		Thread posupdate = new Thread() {
			@Override
			public void run() {
				Thread.currentThread().setName("RealisticGpsProvider");
				se.registerThread(this);

				while(true) {
                    // iterate through all types contained
                    for (Map<String, TrackedModel<Model>> submap : models.values()) {
                        synchronized(submap) {
                            // iterate through all Models of a specific type
                            for (TrackedModel<Model> r : submap.values()) {
                                r.updatePos();
                                receivers.get(r.getName()).receivePosition(r.cur.inMotion());
                            }
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

	private class TrackedModel<T extends Model>{
		//private boolean stopMoving = false;
		private T cur = null;
		private long timeLastUpdate = 0;

		public TrackedModel(T pos) {
			this.cur = pos;
			timeLastUpdate = se.getTime();
		}
		public void updatePos() {
			double timeSinceUpdate = (se.getTime() - timeLastUpdate)/1000.0;

			Point3i p_point = cur.predict(noises, timeSinceUpdate);
			boolean collided = checkCollision(p_point);
			//boolean collided = false; todo(tim) address collisions
			cur.updatePos(!collided);

			cur.updateSensor(obspoint_positions, sensepoint_positions);

			timeLastUpdate = se.getTime();
		}

        public boolean checkCollision(Point3i bot) {
            //double min_distance = Double.MAX_VALUE;
            int myRadius = cur.radius();

            boolean toReturn = false;

            for (Model current : modelPositions.get(cur.getTypeName())) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current.getPos()) <= myRadius + current.radius()) {
                        //update sensors for both robots
                        current.collision(cur.getPos());
                        cur.collision(current.getPos());
                        toReturn = true;
                    }
                }
            }

            ObstacleList list = obspoint_positions;
            for (int i = 0; i < list.ObList.size(); i++) {
                Obstacles currobs = list.ObList.get(i);
                Point3i nextpoint;
                Point3i curpoint;
                ItemPosition wall = new ItemPosition("wall", 0, 0, 0);

                for (int j = 0; j < currobs.obstacle.size(); j++) {
                    curpoint = currobs.obstacle.get(j);
                    if (j == currobs.obstacle.size() - 1) {
                        nextpoint = currobs.obstacle.firstElement();
                    } else {
                        nextpoint = currobs.obstacle.get(j + 1);
                    }
                    Point3i closeP = currobs.getClosestPointOnSegment(curpoint, nextpoint, bot);
                    wall.setPos(closeP);
                    double distance = closeP.distanceTo2D(bot);

                    //need to modify some conditions of bump sensors, we have left and right bump sensor for now
                    if (distance < myRadius) {
                        //update the bump sensor
                        cur.collision(wall.getPos());
                        toReturn = true;
                    }
                }
            }
            if (!toReturn) {
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
    @Override
    public synchronized void halt(String name){}
}
