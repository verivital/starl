package edu.illinois.mitra.starl.harness;

import java.util.HashMap;
import java.util.Observable;
import java.util.Observer;
import java.util.Random;
import java.util.Vector;

import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starl.models.Model_Mavic;
import edu.illinois.mitra.starl.models.Model_Phantom;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.models.Model_3DR;
import edu.illinois.mitra.starl.objects.*;

public class IdealSimGpsProvider extends Observable implements SimGpsProvider  {	
	private HashMap<String, SimGpsReceiver> receivers;
	private HashMap<String, TrackedRobot> robots;

	// Waypoint positions and robot positions that are shared among all robots
	private PositionList<Model_iRobot> robot_positions;
	private PositionList<ItemPosition> waypoint_positions;
	private PositionList<ItemPosition> sensepoint_positions;

    // TD_NATHAN: see what's changed in RealisticSimGpsProvider and emulate similarly?
    //private PositionList<ItemPosition> allpos;
	
	private ObstacleList obspoint_positions;
	private Vector<ObstacleList> viewsOfWorld;
	
	private long period = 100;
	private int angleNoise = 0;
	private int posNoise = 0;

	private Random rand;
	
	private SimulationEngine se;
		
	public IdealSimGpsProvider(SimulationEngine se, long period, double angleNoise, double posNoise) {
		this.se = se;
		this.period = period;
		this.angleNoise = (int) angleNoise;
		this.posNoise = (int) posNoise;
		this.rand = new Random();
		
		receivers = new HashMap<String, SimGpsReceiver>();
		robots = new HashMap<String, TrackedRobot>();
		
		robot_positions = new PositionList<Model_iRobot>();
		waypoint_positions = new PositionList<ItemPosition>();
		sensepoint_positions = new PositionList<ItemPosition>();
	}
	
	@Override
	public synchronized void registerReceiver(String name, SimGpsReceiver simGpsReceiver) {
		receivers.put(name, simGpsReceiver);
	}


	@Override
	public synchronized void setDestination(String name, ItemPosition dest, int velocity) {
		robots.get(name).setDest(dest, velocity);
	}

	@Override
	public synchronized void halt(String name) {
		robots.get(name).setDest(null, 1);
	}
	
	@Override
	public PositionList<Model_iRobot> getiRobotPositions() {
		return robot_positions;
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
    public PositionList<Model_quadcopter> getQuadcopterPositions() {
        // TD_NATHAN: resolve as necessary
        return null;
    }

	@Override
	public PositionList<Model_3DR> get3DRPositions() {
		// TD_NATHAN: resolve as necessary
		return null;
	}

	@Override
	public PositionList<Model_GhostAerial> getGhostAerialsPositions() {
		return null;
	}

	@Override
	public PositionList<Model_Mavic> getMavicPositions(){
		return null;
	}

	@Override
	public PositionList<Model_Phantom> getPhantomPositions(){
		return null;
	}

	@Override
    public PositionList<ItemPosition> getAllPositions() {
        // TD_NATHAN: resolve if necessary
        return null;
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
	public Vector<ObstacleList> getViews() {
		return viewsOfWorld;
	}

	
	
	@Override
	public void start() {
		// Create a periodic runnable which repeats every "period" ms to report positions
		Thread posupdate = new Thread() {
			@Override
			public void run() {
				Thread.currentThread().setName("IdealGpsProvider");
				se.registerThread(this);
				
				while(true) {					
					for(edu.illinois.mitra.starl.harness.IdealSimGpsProvider.TrackedRobot r : robots.values()) {
						if(r.inMotion()) {
							r.updatePos();
							receivers.get(r.getName()).receivePosition(r.inMotion());	
						}
					}	
					setChanged();

					notifyObservers(robot_positions);
					
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
		// Don't notify any observers of null data
		if(data != null) super.notifyObservers(data);
	}

	private class TrackedRobot {
		private int velocity = 200; // TODO was 0
		private ItemPosition start = null;
		private Model_iRobot pos = null;
		private ItemPosition dest = null;
		private boolean newdest = false;
		private boolean reportpos = false;
		private long timeLastUpdate = 0;		
		private long totalMotionTime = 0;
		private long totalTimeInMotion = 0;
		private double motAngle = 0;
		private double vX = 0;
		private double vY = 0;
		private int aNoise = 0;
		private int xNoise = 0;
		private int yNoise = 0;
				
		public TrackedRobot(Model_iRobot pos) {
			this.pos = pos;
			timeLastUpdate = se.getTime();
		}
		public synchronized  void updatePos() {
			
			long timeSinceUpdate = (se.getTime() - timeLastUpdate);
			if(newdest) {
				// Snap to heading
				// Calculate angle and X/Y velocities
				if (start == null)
					throw new RuntimeException("start is null in updatePos()");
				
				int angle = 0;
				
				if (dest == null)
				{
					motAngle = 0;
					angle = 0;
					vX = 0;
					vY = 0;
				}
				else
				{
					int deltaX = dest.x() - start.x();
					int deltaY = dest.y() - start.y();
					motAngle = Math.atan2(deltaY, deltaX);
					
					vX = (Math.cos(motAngle) * velocity);
					vY = (Math.sin(motAngle) * velocity);
					
					// Set position to ideal angle +/- noise
					angle = (int)Math.toDegrees(Math.atan2(deltaY, deltaX));
				}
				
				if(angleNoise != 0) aNoise = rand.nextInt(angleNoise*2)-angleNoise;
				pos.set(start.x(), start.y(), angle+aNoise);
				newdest = false;
			} else if(dest != null) {
				// Calculate noise
				if(angleNoise != 0) aNoise = rand.nextInt(angleNoise*2)-angleNoise;
				if(posNoise != 0) {
					xNoise = rand.nextInt(posNoise*2) - posNoise;
					yNoise = rand.nextInt(posNoise*2) - posNoise;
				}
				// Determine how far we've traveled since the motion started
				// If we've been traveling for longer than it should take to reach the
				// destination, set position to destination and assume we're now at rest. 
				totalTimeInMotion += timeSinceUpdate;
				if(totalTimeInMotion < totalMotionTime) {
					int dX = (int)(vX * totalTimeInMotion)/1000;
					int dY = (int)(vY * totalTimeInMotion)/1000;
					pos.set(start.x() +dX+xNoise, start.y() +dY+yNoise, (int)Math.toDegrees(motAngle));
					pos.velocity = velocity;
				} else {
					
				
					if (dest==null )
					{
						throw new RuntimeException("dest is null");
					}
					
					if (pos==null )
						throw new RuntimeException("pos is null");
					
					pos.set(dest.x() +xNoise, dest.y() +yNoise, (int)pos.angle+aNoise);
					
					dest = null;
					reportpos = true;
				}
			} else {
				reportpos = false;
			}
			timeLastUpdate = se.getTime();
		}
		public synchronized void setDest(ItemPosition dest, int velocity) 
		{
			if (velocity <= 0)
				throw new RuntimeException("setDest called with velocity <= 0");
			
			if(hasChanged()) updatePos();
			
			this.dest = dest;
			this.start = new ItemPosition(pos);
			this.velocity = velocity;
			
			totalMotionTime = (int)(this.start.distanceTo(dest)*1000.0)/velocity;
			totalTimeInMotion = 0;
			newdest = (dest != null);
		}
		public boolean hasChanged() {
			if(reportpos || inMotion()) {
				reportpos = false;
				return true;
			}
			return false;
		}
		public boolean inMotion() {
			return dest != null;
		}
		public String getName() {
			return pos.name;
		}
	}

	@Override
	public void setVelocity(String name, int fwd, int radial) {
		throw new RuntimeException("IdealSimGpsProvider does not use the setVelocity method, but the setDestination method. " +
				"Ideal motion does not use the motion automaton something went very wrong here.");
	}

	@Override
	public void addObserver(Observer o) {
		super.addObserver(o);
	}

	@Override
	public PositionList<ItemPosition> getSensePositions() {
		// TODO Auto-generated method stub
		return null;
	}

    @Override
    public void setControlInput(String name, double v_yaw, double pitch, double roll, double gaz) {
        // TD_NATHAN: fix
        // TODO: replace with PID model here
        //((Model_quadcopter) quadcopters.get(name).cur).v_yawR = v_yaw;
        //((Model_quadcopter) quadcopters.get(name).cur).pitchR = pitch;
        //((Model_quadcopter) quadcopters.get(name).cur).rollR = roll;
        //((Model_quadcopter) quadcopters.get(name).cur).gazR = gaz;
    }

	@Override
	public void setControlInputGA(String name, double v_yaw, double pitch, double roll, double gaz) {}

	@Override
	public void setControlInputMav(String name, double v_yaw, double pitch, double roll, double gaz) {}

	@Override
	public void setControlInputPhantom(String name, double v_yaw, double pitch, double roll, double gaz) {}

	@Override
	public void setControlInput3DR(String name, double v_yaw, double pitch, double roll, double gaz) {}

    /*
    // TD_NATHAN: old version
    @Override
    public synchronized void addRobot(Model_iRobot bot) {
        robots.put(bot.name, new TrackedRobot(bot));
        robot_positions.update(bot);
    }
    */

    @Override
    public synchronized void addRobot(edu.illinois.mitra.starl.interfaces.TrackedRobot bot) {
        // TD_NATHAN: fix
        /*
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
        }
        else{
            throw new RuntimeException("after adding a new model, one need to add model handling in simulation under RealisticSimGpsProvider");
        }
*/
    }
}
