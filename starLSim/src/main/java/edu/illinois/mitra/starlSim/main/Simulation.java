package edu.illinois.mitra.starlSim.main;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Observable;
import java.util.Observer;
import java.util.Random;
import java.util.Set;
import java.util.Vector;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import javax.swing.JFrame;

import edu.illinois.mitra.starl.harness.RealisticSimGpsProvider;
import edu.illinois.mitra.starl.harness.SimGpsProvider;
import edu.illinois.mitra.starl.harness.SimulationEngine;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.ModelRegistry;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PositionList;
import edu.illinois.mitra.starlSim.draw.DrawFrame;
import edu.illinois.mitra.starlSim.draw.RobotData;

public class Simulation {
	private static final double BOT_SPACING_FACTOR = 2.8;
	private Collection<SimApp> bots = new HashSet<SimApp>();
	private HashMap<String, String> participants = new HashMap<String, String>();
	private SimGpsProvider gps;
	private SimulationEngine simEngine;
	private ExecutorService executor;
	private final SimSettings settings;
	private final DrawFrame drawFrame;
	private ObstacleList list;
	private List<List<Object>> resultsList = new ArrayList<List<Object>>();
	private Map<String, ItemPosition> startingPositions = new HashMap<String, ItemPosition>();

	public Simulation(Class<? extends LogicThread> app, final SimSettings settings) {
		// Make sure there is at least one robot entered in settings
		int N_ROBOTS = 0, N_GROUND_BOTS = 0;
		for (Map.Entry<String, SimSettings.Bot> entry : settings.BOTS.entrySet()) {
			int count = entry.getValue().COUNT;
			// Count all robots
			N_ROBOTS += count;
			if (ModelRegistry.isInstance(entry.getKey(), Model_Ground.class)) {
				// Count ground robots for views
				N_GROUND_BOTS += count;
			}
		}
		if (N_ROBOTS == 0) {
			throw new IllegalArgumentException("Must have more than zero robots to simulate!");
		}

		// Create set of robots whose wireless is blocked for passage between
		// the GUI and the simulation communication object
		Set<String> blockedRobots = new HashSet<String>();

		// Create participants and instantiate SimApps
		createParticipants(settings.BOTS);

		// Start the simulation engine
		LinkedList<LogicThread> logicThreads = new LinkedList<LogicThread>();
		if (settings.DRAW) {
			// Initialize viewer
			drawFrame = new DrawFrame(participants.keySet(), blockedRobots, settings);
			drawFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			simEngine = new SimulationEngine(settings.SIM_TIMEOUT, settings.MSG_MEAN_DELAY, settings.MSG_STDDEV_DELAY, settings.MSG_LOSSES_PER_HUNDRED, settings.MSG_RANDOM_SEED, settings.TIC_TIME_RATE, blockedRobots, participants, drawFrame.getPanel(), logicThreads);
		} else {
			drawFrame = null;
			simEngine = new SimulationEngine(settings.SIM_TIMEOUT, settings.MSG_MEAN_DELAY, settings.MSG_STDDEV_DELAY, settings.MSG_LOSSES_PER_HUNDRED, settings.MSG_RANDOM_SEED, settings.TIC_TIME_RATE, blockedRobots, participants, null, logicThreads);

		}


		// Create the sim gps
		// TODO: need to redefine the noises for models in general
		gps = new RealisticSimGpsProvider(simEngine, settings.GPS_PERIOD, settings.GPS_ANGLE_NOISE, settings.GPS_POSITION_NOISE);

		// Load waypoints
		if (settings.WAYPOINT_FILE != null)
			gps.setWaypoints(WptLoader.loadWaypoints(settings.WAYPOINT_FILE));

		// Load sensepoints
		if (settings.SENSEPOINT_FILE != null)
			gps.setSensepoints(SptLoader.loadSensepoints(settings.SENSEPOINT_FILE));

		// Load Obstacles
		if (settings.OBSPOINT_FILE != null) {
			gps.setObspoints(ObstLoader.loadObspoints(settings.OBSPOINT_FILE));
			list = gps.getObspointPositions();
			list.detect_Precision = settings.Detect_Precision;
			list.de_Radius = settings.De_Radius;
			//should we grid the environment?
			if (settings.Detect_Precision > 1) {
				list.Gridfy();
			}
			gps.setViews(list, N_GROUND_BOTS);
		} else {
			//if we have no input files, we still have to initialize the obstacle list so that later on, if we detect collision between robots, we can add that obstacle
			gps.setObspoints(new ObstacleList());
			list = gps.getObspointPositions();
			list.detect_Precision = settings.Detect_Precision;
			list.de_Radius = settings.De_Radius;
			gps.setViews(list, N_GROUND_BOTS);
		}

		this.settings = settings;
		simEngine.setGps(gps);
		gps.start();

		// Load initial positions
		PositionList<ItemPosition> t_initialPositions;
		if (settings.INITIAL_POSITIONS_FILE != null) {
			t_initialPositions = WptLoader.loadWaypoints(settings.INITIAL_POSITIONS_FILE);
		} else
			t_initialPositions = new PositionList<>();

		// Create each robot
		for (Map.Entry<String, SimSettings.Bot> entry : settings.BOTS.entrySet()) {
			int count = entry.getValue().COUNT;
			String name = entry.getValue().NAME;
			String typeName = entry.getKey();
			addModels(count, name, t_initialPositions, app, logicThreads, typeName);
		}

		if (settings.USE_GLOBAL_LOGGER)
			gps.addObserver(createGlobalLogger(settings));

		//**********************************************************************************************************************************
		//gps.addObserver(addChristinaObserver(settings));
		//**********************************************************************************************************************************

		if (settings.DRAW) {
			// initialize debug drawer class if it was set in the settings
			if (settings.DRAWER != null)
				drawFrame.addPredrawer(settings.DRAWER);
			// GUI observer updates the viewer when new positions are calculated
			Observer guiObserver = new Observer() {
				@Override
				public void update(Observable o, Object arg) {
					Vector<ObstacleList> views = gps.getViews();
					ArrayList<RobotData> rd = new ArrayList<RobotData>();
					if (!(arg instanceof PositionList)) {
						return;
					}
					PositionList<? extends ItemPosition> argList = (PositionList<? extends ItemPosition>)arg;
					ArrayList<? extends ItemPosition> targetList = argList.getList();

					for (int i = 0, iGround = 0; i < targetList.size(); i++) {
						ItemPosition ip = targetList.get(i);
						RobotData nextBot;
						if (ip instanceof Model_Ground) {
							//tracks ith ground robot since only ground robots access the views vector
							nextBot = new RobotData(ip, Color.black, views.elementAt(iGround));
							iGround++;
						} else {
							nextBot = new RobotData(ip);
						}
						rd.add(nextBot);
					}

					// Add waypoints
					if (settings.DRAW_WAYPOINTS) {
						for (ItemPosition ip : gps.getWaypointPositions().getList()) {
							RobotData waypoint = new RobotData(ip, Color.red);
							rd.add(waypoint);
						}
					}
					drawFrame.updateData(rd, simEngine.getTime());
					//add obstacle update later
				}
			};
			gps.addObserver(guiObserver);
			// show viewer
			drawFrame.setVisible(true);
		}
	}

	/**
	 * Add an Observer to the list of GPS observers. This Observer's update
	 * method will be passed a PositionList object as the argument. This must be
	 * called before the simulation is started!
	 *
	 * @param o
	 */
	public void addPositionObserver(Observer o) {
		if (executor == null)
			gps.addObserver(o);
	}

	/**
	 * Begins executing a simulation. This call will block until the simulation completes.
	 */
	public void start() {
		executor = Executors.newFixedThreadPool(participants.size());
		// Save settings to JSON file
		if(settings.TRACE_OUT_DIR != null)
			SettingsWriter.writeSettings(settings);

		// Invoke all simulated robots
		List<Future<List<Object>>> results = null;
		try {
			if(settings.TIMEOUT > 0)
				results = executor.invokeAll(bots, settings.TIMEOUT, TimeUnit.SECONDS);
			else
				results = executor.invokeAll(bots);
		} catch(InterruptedException e) {
			e.printStackTrace();
		}

		// Wait until all result values are available
		if (results != null) {
			for (Future<List<Object>> f : results) {
				try {
					List<Object> res = f.get();
					if (res != null && !res.isEmpty())
						resultsList.add(res);
				} catch (CancellationException e) {
					// If the executor timed out, the result is cancelled
					System.err.println("Simulation timed out! Execution reached "
							+ settings.TIMEOUT + " sec duration. Aborting.");
					break;
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}

		shutdown();
	}

	public void shutdown() {
		simEngine.simulationDone();
		executor.shutdownNow();
	}

	public void closeWindow() {
		drawFrame.dispose();
	}

	public List<List<Object>> getResults() {
		return resultsList;
	}

	public long getSimulationDuration() {
		return simEngine.getDuration();
	}

	public String getMessageStatistics() {
		return 	simEngine.getComChannel().getStatistics();
	}

	private boolean acceptableStart(ItemPosition pos) {
		// does not modify this
		if (pos == null)
			return false;
		final int minDistSq = (int)Math.ceil(Math.pow(BOT_SPACING_FACTOR * settings.BOT_RADIUS, 2));
		for (Entry<String, ItemPosition> entry : startingPositions.entrySet()) {
			if (entry.getValue().getPos().subtract(pos.getPos()).magnitudeSq() < minDistSq)
				return false;
		}
		return true;
	}

	private Observer createGlobalLogger(final SimSettings settings) {
		final GlobalLogger gl = new GlobalLogger(settings.TRACE_OUT_DIR, "global.txt");
		System.out.println("SANITY CHECK createGlobalLogger called");

		// global logger observer updates the log file when new positions are calculated
		Observer globalLogger = new Observer() {
			@Override
			public void update(Observable o, Object arg) {
				ArrayList<RobotData> rd = new ArrayList<RobotData>();
				ArrayList<ItemPosition> pos = ((PositionList) arg).getList();
				for(ItemPosition ip : pos) {
					RobotData nextBot = new RobotData(ip);
					rd.add(nextBot);
				}
				gl.updateData(rd, simEngine.getTime());
			}
		};
		return globalLogger;
	}

	private void createParticipants(Map<String, SimSettings.Bot> bots) {
		int index = 0;
		for (Map.Entry<String, SimSettings.Bot> entry : bots.entrySet()) {
			// use a temporary to get IP address
			String ip = ModelRegistry.create(entry.getKey()).ip();
			for (int i = 0; i < entry.getValue().COUNT; i++) {
				// Mapping between robot name and IP address
				participants.put(entry.getValue().NAME + i, ip + (i + index));
				index++;
			}
		}
	}

	private void addModels(int count, String name, PositionList<ItemPosition> t_initialPositions,
						   Class<? extends LogicThread> app, List<LogicThread> logicThreads,
						   String typeName) {
		Random rand = new Random();
		for (int i = 0; i < count; i++) {
			Model model = null;
			String botName = name + i;
			ItemPosition initialPos = t_initialPositions.getPosition(botName);
			if (initialPos != null) {
				// instantiate a subclass of Model without knowing the exact type
				model = ModelRegistry.create(typeName, initialPos);
			} else {
				// If no initial position was supplied, randomly generate one
				// Must pass both acceptableStart() and list.validstarts()
				boolean valid;
				int retries = 0;
				Integer radius = null;
				do {
					initialPos = new ItemPosition(botName, rand.nextInt(settings.GRID_XSIZE),
							rand.nextInt(settings.GRID_YSIZE));
					valid = acceptableStart(initialPos);
					if (valid) {
						if (radius == null) {
							// radius saved after first time, avoid expensive call to create model
							model = ModelRegistry.create(typeName, initialPos);
							radius = model.radius();
						}
						valid = list.validstarts(initialPos, radius);
					}
					if (valid) {
						if (model == null) {
							model = ModelRegistry.create(typeName, initialPos);
						}
					} else {
						model = null; // dispose of invalid model
					}
					retries++;
				} while (!valid && retries < 10000);
				if (!valid) {
					throw new RuntimeException("too many tries for BOT" + botName + "please increase settings.GRID_XSIZE/GRID_YSIZE or remove some obstacles");
				}
			}

			// Add type for ground models
			if (model instanceof Model_Ground) {
				Model_Ground modelGround = (Model_Ground)model;
				if (i < settings.N_DISCOV_BOTS) {
					modelGround.type = Model_Ground.Type.EXPLORE_AREA;
				} else if ((i >= settings.N_DISCOV_BOTS) && (i < (settings.N_DISCOV_BOTS + settings.N_RAND_BOTS))) {
					modelGround.type = Model_Ground.Type.RANDOM_MOVING_OBSTACLE;
				} else {
					modelGround.type = Model_Ground.Type.GET_TO_GOAL;
					//default robot type is get to goal
				}
			}

			SimApp sa = new SimApp(botName, participants, simEngine, model, settings.TRACE_OUT_DIR,
					app, drawFrame, settings.TRACE_CLOCK_DRIFT_MAX, settings.TRACE_CLOCK_SKEW_MAX);
			bots.add(sa);
			logicThreads.add(sa.logic);
			simEngine.addLogging(sa.gvh.log);
		}
	}
}
