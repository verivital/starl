package edu.illinois.mitra.starl.harness;

import java.util.HashMap;
import java.util.Map;
import java.util.Observable;
import java.util.Observer;
import java.util.Vector;
import java.util.concurrent.ConcurrentHashMap;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_3DR;
import edu.illinois.mitra.starl.models.Model_Drone;
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

    /*
     * Accesses or creates a map of Models of the specified typename. Replaces previous
     *     private Map<String, TrackedModel<Model_iRobot>> iRobots;
     *     ...
     *     private Map<String, TrackedModel<Model_Phantom>> phantoms;
     */
    private Map<String, TrackedModel<Model>> getModels(String typename) {
        Map<String, TrackedModel<Model>> subMap = models.get(typename);
        if (subMap == null) {
            subMap = models.put(typename, new HashMap<>());
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
            list = modelPositions.put(typename, new PositionList<>());
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

        models = new HashMap<>();
        modelPositions = new HashMap<>();
        receivers = new HashMap<String, SimGpsReceiver>();

		/*iRobots = new ConcurrentHashMap<String, TrackedModel<Model_iRobot>>();
		quadcopters = new ConcurrentHashMap<String, TrackedModel<Model_quadcopter>>();
		ghosts = new ConcurrentHashMap<String, TrackedModel<Model_GhostAerial>>();
		mavics = new ConcurrentHashMap<String, TrackedModel<Model_Mavic>>();
		phantoms = new ConcurrentHashMap<String, TrackedModel<Model_Phantom>>();
		o3DRs = new ConcurrentHashMap<String, TrackedModel<Model_3DR>>();*/

        waypoint_positions = new PositionList<ItemPosition>();
        sensepoint_positions = new PositionList<ItemPosition>();
        allpos = new PositionList<ItemPosition>();

		/*iRobot_positions = new PositionList<Model_iRobot>();
		quadcopter_positions = new PositionList<Model_quadcopter>();
		ghost_positions = new PositionList<Model_GhostAerial>();
		mavic_positions = new PositionList<Model_Mavic>();
		phantom_positions = new PositionList<Model_Phantom>();
		o3DR_positions = new PositionList<Model_3DR>();*/
    }

    @Override
    public synchronized void registerReceiver(String name, SimGpsReceiver simGpsReceiver) {
        receivers.put(name, simGpsReceiver);
    }

    @Override
    public synchronized void addRobot(Model bot) {
        allpos.update(bot);

        String typeName = bot.getTypeName();
        synchronized (models) {
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
    public void setVelocity(String typeName, String name, int fwd, int rad) {
        ((Model_iRobot) models.get(typeName).get(name).cur).vFwd = fwd;
        ((Model_iRobot) models.get(typeName).get(name).cur).vRad = rad;
    }

    @Override
    public void setControlInput(String typeName, String name, double v_yaw, double pitch, double roll, double gaz) {
        /** TODO: replace with PID model here
         */

        Model_Drone drone = (Model_Drone) models.get(typeName).get(name).cur;
        drone.v_yawR = v_yaw;
        drone.pitchR = pitch;
        drone.rollR = roll;
        drone.gazR = gaz;
    }

    @Override
    public PositionList<ItemPosition> getAllPositions() {
        return allpos;
    }

    @Override
    public void setWaypoints(PositionList<ItemPosition> loadedWaypoints) {
        if (loadedWaypoints != null) waypoint_positions = loadedWaypoints;
    }

    @Override
    public void setSensepoints(PositionList<ItemPosition> loadedSensepoints) {
        if (loadedSensepoints != null) sensepoint_positions = loadedSensepoints;
    }

    @Override
    public void setObspoints(ObstacleList loadedObspoints) {
        if (loadedObspoints != null) obspoint_positions = loadedObspoints;
    }


    @Override
    public void setViews(ObstacleList environment, int nBots) {
        if (environment != null) {
            viewsOfWorld = new Vector<ObstacleList>(3, 2);
            ObstacleList obsList = null;
            for (int i = 0; i < nBots; i++) {
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

                while (true) {
                    // iterate through all types contained
                    for (Map<String, TrackedModel<Model>> submap : models.values()) {
                        synchronized (submap) {
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
        if (data != null) super.notifyObservers(data);
    }

    private class TrackedModel<T extends Model> {
        //private boolean stopMoving = false;
        private T cur = null;
        private long timeLastUpdate = 0;

        public TrackedModel(T pos) {
            this.cur = pos;
            timeLastUpdate = se.getTime();
        }

        public void updatePos() {
            double timeSinceUpdate = (se.getTime() - timeLastUpdate) / 1000.0;

            Point3d p_point = cur.predict(noises, timeSinceUpdate);
            boolean collided = checkCollision(p_point);
            //boolean collided = false; todo(tim) address collisions
            cur.updatePos(!collided);

            cur.updateSensor(obspoint_positions, sensepoint_positions);

            timeLastUpdate = se.getTime();
        }

        public boolean checkCollision(Point3d bot) {
            //double min_distance = Double.MAX_VALUE;
            int myRadius = cur.radius();

            /*if (cur instanceof Model_iRobot) {
                myRadius = ((Model_iRobot) cur).radius;
            } else if (cur instanceof Model_quadcopter) {
                myRadius = ((Model_quadcopter) cur).radius;
            } else if (cur instanceof Model_GhostAerial) {
                myRadius = ((Model_GhostAerial) cur).radius;
            } else if (cur instanceof Model_Mavic) {
                myRadius = ((Model_Mavic) cur).radius;
            } else if (cur instanceof Model_Phantom) {
                myRadius = ((Model_Phantom) cur).radius;
            } else if (cur instanceof Model_3DR) {
                myRadius = ((Model_3DR) cur).radius;
            } else {
                throw new RuntimeException("after adding a new model, one need to add collision handling under RealisticSimGpsProvider");
            }*/

            boolean toReturn = false;

            for (Model current : modelPositions.get(cur.getTypeName())) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current) <= myRadius + current.radius()) {
                        //update sensors for both robots
                        current.collision(cur);
                        cur.collision(current);
                        toReturn = true;
                    }
                    //min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
                }
            }

            /*for (Model_iRobot current : iRobot_positions.getList()) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current) <= myRadius + current.radius) {
                        //update sensors for both robots
                        current.collision(cur);
                        cur.collision(current);
                        toReturn = true;
                    }
                    //min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
                }
            }



            for (Model_GhostAerial current : ghost_positions.getList()) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current) <= myRadius + current.radius) {
                        //update sensors for both robots
                        current.collision(cur);
                        cur.collision(current);
                        toReturn = true;
                    }
                    //min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
                }
            }

            for (Model_Mavic current : mavic_positions.getList()) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current) <= myRadius + current.radius) {
                        //update sensors for both robots
                        current.collision(cur);
                        cur.collision(current);
                        toReturn = true;
                    }
                    //min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
                }
            }
            for (Model_Phantom current : phantom_positions.getList()) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current) <= myRadius + current.radius) {
                        //update sensors for both robots
                        current.collision(cur);
                        cur.collision(current);
                        toReturn = true;
                    }
                    //min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
                }
            }
            for (Model_3DR current : o3DR_positions.getList()) {
                if (!current.name.equals(cur.name)) {
                    if (bot.distanceTo(current) <= myRadius + current.radius) {
                        //update sensors for both robots
                        current.collision(cur);
                        cur.collision(current);
                        toReturn = true;
                    }
                    //min_distance = Math.min(bot.distanceTo(current) - current.radius, min_distance);
                }
            }*/

            ObstacleList list = obspoint_positions;
            for (int i = 0; i < list.ObList.size(); i++) {
                Obstacles currobs = list.ObList.get(i);
                Point3d nextpoint = currobs.obstacle.firstElement();
                Point3d curpoint = currobs.obstacle.firstElement();
                ItemPosition wall = new ItemPosition("wall", 0, 0, 0);

                for (int j = 0; j < currobs.obstacle.size(); j++) {
                    curpoint = currobs.obstacle.get(j);
                    if (j == currobs.obstacle.size() - 1) {
                        nextpoint = currobs.obstacle.firstElement();
                    } else {
                        nextpoint = currobs.obstacle.get(j + 1);
                    }
                    Point3d closeP = currobs.getClosestPointOnSegment(curpoint.x, curpoint.y, nextpoint.x, nextpoint.y, bot.x, bot.y);
                    wall.setPos(closeP.x, closeP.y, 0);
                    double distance = Math.sqrt(Math.pow(closeP.x - bot.x, 2) + Math.pow(closeP.y - bot.y, 2));

                    //need to modify some conditions of bump sensors, we have left and right bump sensor for now
                    if (distance < myRadius) {
                        //update the bump sensor
                        cur.collision(wall);
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
