package edu.illinois.mitra.starlSim.main;

import edu.illinois.mitra.starlSim.draw.Drawer;

/**
 * @author Adam
 * 
 */
public class SimSettings {
	
	//grid size of the obstacle representations
	public final int Detect_Precision;
	
	//mark a rectangle when hit an unknown obstacle, length of rectangle = De_Radadius * Detect_Precision
	// may change to mark a circle later on
	public final int De_Radius;
	
	/**
	 * The number of iRobots to simulate.
	 */
	public final int N_IROBOTS;
	
	/**
	 * The number of get to the goal rRobots to simulate.
	 */
	public final int N_GBOTS;
	
	/**
	 * The number of discovery iRobots to simulate.
	 */
	public final int N_DBOTS;
	
	/**
	 * The number of random moving iRobots to simulate.
	 */
	public final int N_RBOTS;
	
	/**
	 * The number of quadcopters to simulate.
	 */
	public final int N_QUADCOPTERS;

	/**
	 * The number of Ghost Aerials to simulate.
	 */
	public final int N_GHOSTS;

    /**
     * The number of mavics to simulate.
     */
    public final int N_MAVICS;

	/**
	 * The number of Phantoms to simulate.
	 */
	public final int N_PHANTOMS;

	/**
	 * The number of 3dr solo to simulate.
	 */
	public final int N_o3DR;
	
	/**
	 * Default 0.
	 * The maximum number of seconds (real time) the simulation may be executing
	 * for. Setting to zero will allow simulations to run indefinitely.
	 */
	public final long TIMEOUT;

	/**
	 * Default 0.
	 * The maximum number of milliseconds (simulated time) the simulation may
	 * execute for before timing out. Setting to zero will allow simulations to
	 * run indefinitely.
	 */
	public final long SIM_TIMEOUT;

	/**
	 * Filename for a .wpt file with waypoints.
	 */
	public final String WAYPOINT_FILE;	/**

	 * Filename for a .wpt file with SensePoints.
	 */
	public final String SENSEPOINT_FILE;	/**

	 * Filename for a .wpt file with Obstacles.
	 */
	public final String OBSPOINT_FILE;

	/**
	 * Filename for a .wpt file with initial positions for robots, or null to
	 * enable random starting locations.
	 */
	public final String INITIAL_POSITIONS_FILE;

	/**
	 * Enable/disable ideal motion. False uses simulated motion automaton, true
	 * uses unrealistic motion model
	 */
	public final boolean IDEAL_MOTION;

	/**
	 * The desired rate of time passing. 0 = no limit, 0.5 = half real-time, 1.0
	 * = real-time, etc.
	 */
	public final double TIC_TIME_RATE;

	/**
	 * Simulated world width.
	 */
	public final int GRID_XSIZE;
	/**
	 * Simulated world height.
	 */
	public final int GRID_YSIZE;
	/**
	 * Simulated world depth.
	 */
	public final int GRID_ZSIZE;
	
	/**
	 * Milliseconds. The time between simulated GPS position broadcasts.
	 */
	public final long GPS_PERIOD;
	/**
	 * Degrees. The maximum angular noise of simulated GPS positions.
	 */
	public final double GPS_ANGLE_NOISE;
	/**
	 * Millimeters. The maximum position X and Y offset of simulated GPS
	 * positions.
	 */
	public final double GPS_POSITION_NOISE;

	/**
	 * Milliseconds. The maximum trace clock drift.
	 */
	public final int TRACE_CLOCK_DRIFT_MAX;
	/**
	 * The maximum trace clock skew.
	 */
	public final double TRACE_CLOCK_SKEW_MAX;

	/**
	 * Milliseconds. The average message transit time.
	 */
	public final int MSG_MEAN_DELAY;
	/**
	 * Milliseconds. The standard deviation of message transmission times.
	 */
	public final int MSG_STDDEV_DELAY;
	/**
	 * Number of messages to drop per hundred messages sent.
	 */
	public final int MSG_LOSSES_PER_HUNDRED;
	/**
	 * Seed for random number generator used by the communication channel.
	 */
	public final int MSG_RANDOM_SEED;

	/**
	 * iRobot name prefix
	 */
	public final String IROBOT_NAME;
	
	/**
	 * quadcopter name prefix
	 */
	public final String QUADCOPTER_NAME;

	/**
	 * ghost name prefix
	 */
	public final String GHOST_NAME;

    /**
     * Mavic name prefix
     */
    public final String MAVIC_NAME;

	/**
	 * Mavic name prefix
	 */
	public final String PHANTOM_NAME;

	/**
	 * 3dr solo name prefix
	 */
	public final String o3DR_NAME;
	
	/**
	 * Millimeters. The radius of simulated robots.
	 */
	public final int BOT_RADIUS;

	/**
	 * Trace output directory.
	 */
	public final String TRACE_OUT_DIR;
	/**
	 * enable 3D visualizer
	 */
	public final boolean THREE_D;
	/**
	 * Enable/disable the global logger.
	 */
	public final boolean USE_GLOBAL_LOGGER;

	/**
	 * Enables/disables drawing for a simulation
	 */
	public final boolean DRAW;
	/**
	 * Enables/disables trace drawing
	 */
	public final boolean DRAW_TRACE;
	/**
	 * The trace length for each robot.
	 */
	public final int DRAW_TRACE_LENGTH;

    /**
     * The stroke size to use for robots (larger number = thicker lines, useful for using on slides)
     */
    public final int DRAW_ROBOT_STROKE_SIZE;

	/**
	 * The maximum frames per second to draw at. If drawing takes too long,
	 * lower this.
	 */
	public final int MAX_FPS;

	/**
	 * The object which does the drawing of debug information for the simulator.
	 * Can be null if unused.
	 */
	public final Drawer DRAWER;

	/**
	 * Enables/disables drawing waypoints in the visualizer
	 */
	public final boolean DRAW_WAYPOINTS;
	
	/**
	 * Enables/disables drawing obspoints in the visualizer
	 */
	public final boolean DRAW_OBSPOINTS;

	/**
	 * Enables/disables drawing waypoint names next to each waypoint in the
	 * visualizer. Unaffected by the value of DRAW_WAYPOINTS
	 */
	public final boolean DRAW_WAYPOINT_NAMES;
	
	/**
	 * Enables/disables draw robot type next to a robot
	 */
	public final boolean DRAW_ROBOT_TYPE;


	private static final SimSettings defaultInstance = new Builder().build();

	public static SimSettings defaultSettings() {
		return defaultInstance;
	}

	public static class Builder {
		private int De_Radius = 1;
		private int Detect_Precision = 1;
		private int N_IROBOTS = 4;
		private int N_GBOTS = 4;
		private int N_DBOTS = 0;
		private int N_RBOTS = 0;
		private int N_QUADCOPTERS = 0;
		private int N_GHOSTS = 0;
        private int N_MAVICS = 0;
		private int N_PHANTOMS = 0;
		private int N_o3DR = 0;
		
		private long SIM_TIMEOUT = 0;
		private long TIMEOUT = 0;
		private String WAYPOINT_FILE;
		private String SENSEPOINT_FILE;
		private String OBSPOINT_FILE;
		private String INITIAL_POSITIONS_FILE;
		private boolean IDEAL_MOTION = false;
		private double TIC_TIME_RATE = 5;
		private int GRID_XSIZE = 5000;
		private int GRID_YSIZE = 3000;
		private int GRID_ZSIZE = 10000;
		private long GPS_PERIOD = 75;
		private double GPS_ANGLE_NOISE = 0;
		private double GPS_POSITION_NOISE = 0;
		private int TRACE_CLOCK_DRIFT_MAX = 100;
		private double TRACE_CLOCK_SKEW_MAX = 0.00000015;
		private int MSG_MEAN_DELAY = 15;
		private int MSG_STDDEV_DELAY = 5;
		private int MSG_LOSSES_PER_HUNDRED = 0;
		private int MSG_RANDOM_SEED = 0;
		private String IROBOT_NAME = "iRobot";
		private String QUADCOPTER_NAME = "quadcopter";
		private String GHOST_NAME = "ghostie";
        private String MAVIC_NAME = "mavic";
		private String PHANTOM_NAME = "Phantom";
		private String o3DR_NAME = "Solo";
		private int BOT_RADIUS = 165;
		private String TRACE_OUT_DIR;
		private boolean THREE_D = true;
		private boolean USE_GLOBAL_LOGGER = false;
		private boolean DRAW = true;
		private boolean DRAW_TRACE = false;
		private int DRAW_TRACE_LENGTH = 128;
        private int DRAW_ROBOT_STROKE_SIZE = 20;
		private int MAX_FPS = 30;
		private Drawer DRAWER = null;
		private boolean DRAW_WAYPOINTS = true;
		private boolean DRAW_OBSPOINTS = true;
		private boolean DRAW_WAYPOINT_NAMES = true;
		private boolean DRAW_ROBOT_TYPE = false;

		public int getGRID_XSIZE(){
			return GRID_XSIZE;
		}

		public int getGRID_YSIZE(){
			return GRID_YSIZE;
		}


		public Builder De_Radius(int length) {
			this.De_Radius = length;
			return this;
		}
		
		public Builder Detect_Precision(int sqsize) {
			this.Detect_Precision = sqsize;
			return this;
		}
		
		public Builder N_IROBOTS(int N_IROBOTS) {
			this.N_IROBOTS = N_IROBOTS;
			return this;
		}
		
		public Builder N_GBOTS(int N_GBOTS) {
			this.N_GBOTS = N_GBOTS;
			return this;
		}
		
		public Builder N_DBOTS(int N_DBOTS) {
			this.N_DBOTS = N_DBOTS;
			return this;
		}
		
		public Builder N_RBOTS(int N_RBOTS) {
			this.N_RBOTS = N_RBOTS;
			return this;
		}
		
		public Builder N_QUADCOPTERS(int N_QUADCOPTERS) {
			this.N_QUADCOPTERS = N_QUADCOPTERS;
			return this;
		}

		public Builder N_GHOSTS(int N_GHOSTS) {
			this.N_GHOSTS = N_GHOSTS;
			return this;
		}

        public Builder N_MAVICS(int N_MAVICS) {
            this.N_MAVICS = N_MAVICS;
            return this;
        }
		public Builder N_PHANTOMS(int N_PHANTOMS) {
			this.N_PHANTOMS = N_PHANTOMS;
			return this;
		}

		public Builder N_o3DR(int N_o3DR) {
			this.N_o3DR = N_o3DR;
			return this;
		}
		
		public Builder WAYPOINT_FILE(String WAYPOINT_FILE) {
			this.WAYPOINT_FILE = WAYPOINT_FILE;
			return this;
		}
		public Builder SENSEPOINT_FILE(String SENSEPOINT_FILE) {
			this.SENSEPOINT_FILE = SENSEPOINT_FILE;
			return this;
		}
		public Builder OBSPOINT_FILE(String OBSPOINT_FILE) {
			this.OBSPOINT_FILE = OBSPOINT_FILE;
			return this;
		}

		public Builder SIM_TIMEOUT(long SIM_TIMEOUT) {
			this.SIM_TIMEOUT = SIM_TIMEOUT;
			return this;
		}
		
		public Builder TIMEOUT(long TIMEOUT) {
			this.TIMEOUT = TIMEOUT;
			return this;
		}

		public Builder INITIAL_POSITIONS_FILE(String INITIAL_POSITIONS_FILE) {
			this.INITIAL_POSITIONS_FILE = INITIAL_POSITIONS_FILE;
			return this;
		}

		public Builder IDEAL_MOTION(boolean IDEAL_MOTION) {
			this.IDEAL_MOTION = IDEAL_MOTION;
			return this;
		}

		public Builder TIC_TIME_RATE(double TIC_TIME_RATE) {
			this.TIC_TIME_RATE = TIC_TIME_RATE;
			return this;
		}

		public Builder GRID_XSIZE(int GRID_XSIZE) {
			this.GRID_XSIZE = GRID_XSIZE;
			return this;
		}

		public Builder GRID_YSIZE(int GRID_YSIZE) {
			this.GRID_YSIZE = GRID_YSIZE;
			return this;
		}
		
		public Builder GRID_ZSIZE(int GRID_ZSIZE) {
			this.GRID_ZSIZE = GRID_ZSIZE;
			return this;
		}

		public Builder GPS_PERIOD(long GPS_PERIOD) {
			this.GPS_PERIOD = GPS_PERIOD;
			return this;
		}

		public Builder GPS_ANGLE_NOISE(double GPS_ANGLE_NOISE) {
			this.GPS_ANGLE_NOISE = GPS_ANGLE_NOISE;
			return this;
		}

		public Builder GPS_POSITION_NOISE(double GPS_POSITION_NOISE) {
			this.GPS_POSITION_NOISE = GPS_POSITION_NOISE;
			return this;
		}

		public Builder TRACE_CLOCK_DRIFT_MAX(int TRACE_CLOCK_DRIFT_MAX) {
			this.TRACE_CLOCK_DRIFT_MAX = TRACE_CLOCK_DRIFT_MAX;
			return this;
		}

		public Builder TRACE_CLOCK_SKEW_MAX(double TRACE_CLOCK_SKEW_MAX) {
			this.TRACE_CLOCK_SKEW_MAX = TRACE_CLOCK_SKEW_MAX;
			return this;
		}

		public Builder MSG_MEAN_DELAY(int MSG_MEAN_DELAY) {
			this.MSG_MEAN_DELAY = MSG_MEAN_DELAY;
			return this;
		}

		public Builder MSG_STDDEV_DELAY(int MSG_STDDEV_DELAY) {
			this.MSG_STDDEV_DELAY = MSG_STDDEV_DELAY;
			return this;
		}

		public Builder MSG_LOSSES_PER_HUNDRED(int MSG_LOSSES_PER_HUNDRED) {
			this.MSG_LOSSES_PER_HUNDRED = MSG_LOSSES_PER_HUNDRED;
			return this;
		}

		public Builder MSG_RANDOM_SEED(int MSG_RANDOM_SEED) {
			this.MSG_RANDOM_SEED = MSG_RANDOM_SEED;
			return this;
		}

		public Builder IROBOT_NAME(String IROBOT_NAME) {
			this.IROBOT_NAME = IROBOT_NAME;
			return this;
		}

		public Builder QUADCOPTER_NAME(String QUADCOPTER_NAME) {
			this.QUADCOPTER_NAME = QUADCOPTER_NAME;
			return this;
		}

        public Builder MAVIC_NAME(String MAVIC_NAME) {
			this.MAVIC_NAME = MAVIC_NAME;
			return this;
		}

		public Builder PHANTOM_NAME(String PHANTOM_NAME) {
			this.PHANTOM_NAME = PHANTOM_NAME;
			return this;
		}

		public Builder o3DR_NAME(String o3DR_NAME) {
			this.o3DR_NAME = o3DR_NAME;
			return this;
		}

        public Builder GHOST_NAME(String GHOST_NAME) {
            this.GHOST_NAME = GHOST_NAME;
            return this;
        }

		public Builder BOT_RADIUS(int BOT_RADIUS) {
			this.BOT_RADIUS = BOT_RADIUS;
			return this;
		}

		public Builder TRACE_OUT_DIR(String TRACE_OUT_DIR) {
			this.TRACE_OUT_DIR = TRACE_OUT_DIR;
			return this;
		}

		public Builder THREE_D(boolean THREE_D) {
			this.THREE_D = THREE_D;
			return this;
		}
		
		public Builder USE_GLOBAL_LOGGER(boolean USE_GLOBAL_LOGGER) {
			this.USE_GLOBAL_LOGGER = USE_GLOBAL_LOGGER;
			return this;
		}

		public Builder DRAW(boolean DRAW){
			this.DRAW = DRAW;
			return this;
		}
		
		public Builder DRAW_TRACE(boolean DRAW_TRACE) {
			this.DRAW_TRACE = DRAW_TRACE;
			return this;
		}

		public Builder DRAW_TRACE_LENGTH(int DRAW_TRACE_LENGTH) {
			this.DRAW_TRACE_LENGTH = DRAW_TRACE_LENGTH;
			return this;
		}

        public Builder DRAW_ROBOT_STROKE_SIZE(int DRAW_ROBOT_STROKE_SIZE) {
            this.DRAW_ROBOT_STROKE_SIZE = DRAW_ROBOT_STROKE_SIZE;
            return this;
        }

		public Builder MAX_FPS(int MAX_FPS) {
			this.MAX_FPS = MAX_FPS;
			return this;
		}

		public Builder DRAWER(Drawer DRAWER) {
			this.DRAWER = DRAWER;
			return this;
		}

		public Builder DRAW_WAYPOINTS(boolean DRAW_WAYPOINTS) {
			this.DRAW_WAYPOINTS = DRAW_WAYPOINTS;
			return this;
		}
		
		public Builder DRAW_OBSPOINTS(boolean DRAW_OBSPOINTS) {
			this.DRAW_OBSPOINTS = DRAW_OBSPOINTS;
			return this;
		}

		public Builder DRAW_WAYPOINT_NAMES(boolean DRAW_WAYPOINT_NAMES) {
			this.DRAW_WAYPOINT_NAMES = DRAW_WAYPOINT_NAMES;
			return this;
		}
		
		public Builder DRAW__ROBOT_TYPE(boolean DRAW_ROBOT_TYPE){
			this.DRAW_ROBOT_TYPE = DRAW_ROBOT_TYPE;
			return this;
		}

		public SimSettings build() {
			return new SimSettings(this);
		}
	}

	private SimSettings(Builder builder) {
		this.De_Radius = builder.De_Radius;
		this.Detect_Precision = builder.Detect_Precision;
		this.N_IROBOTS = builder.N_IROBOTS;
		this.N_GBOTS = builder.N_GBOTS;
		this.N_DBOTS = builder.N_DBOTS;
		this.N_RBOTS = builder.N_RBOTS;
		this.N_QUADCOPTERS = builder.N_QUADCOPTERS;
		this.N_GHOSTS = builder.N_GHOSTS;
        this.N_MAVICS = builder.N_MAVICS;
		this.N_PHANTOMS = builder.N_PHANTOMS;
		this.N_o3DR = builder.N_o3DR;
		this.SENSEPOINT_FILE = builder.SENSEPOINT_FILE;
		this.WAYPOINT_FILE = builder.WAYPOINT_FILE;
		this.OBSPOINT_FILE = builder.OBSPOINT_FILE;
		this.INITIAL_POSITIONS_FILE = builder.INITIAL_POSITIONS_FILE;
		this.IDEAL_MOTION = builder.IDEAL_MOTION;
		this.TIC_TIME_RATE = builder.TIC_TIME_RATE;
		this.GRID_XSIZE = builder.GRID_XSIZE;
		this.GRID_YSIZE = builder.GRID_YSIZE;
		this.GRID_ZSIZE = builder.GRID_ZSIZE;
		this.GPS_PERIOD = builder.GPS_PERIOD;
		this.GPS_ANGLE_NOISE = builder.GPS_ANGLE_NOISE;
		this.GPS_POSITION_NOISE = builder.GPS_POSITION_NOISE;
		this.TRACE_CLOCK_DRIFT_MAX = builder.TRACE_CLOCK_DRIFT_MAX;
		this.TRACE_CLOCK_SKEW_MAX = builder.TRACE_CLOCK_SKEW_MAX;
		this.MSG_MEAN_DELAY = builder.MSG_MEAN_DELAY;
		this.MSG_STDDEV_DELAY = builder.MSG_STDDEV_DELAY;
		this.MSG_LOSSES_PER_HUNDRED = builder.MSG_LOSSES_PER_HUNDRED;
		this.MSG_RANDOM_SEED = builder.MSG_RANDOM_SEED;
		this.IROBOT_NAME = builder.IROBOT_NAME;
		this.QUADCOPTER_NAME = builder.QUADCOPTER_NAME;
		this.GHOST_NAME = builder.GHOST_NAME;
        this.MAVIC_NAME = builder.MAVIC_NAME;
		this.PHANTOM_NAME = builder.PHANTOM_NAME;
		this.o3DR_NAME = builder.o3DR_NAME;
		this.BOT_RADIUS = builder.BOT_RADIUS;
		this.TRACE_OUT_DIR = builder.TRACE_OUT_DIR;
		this.THREE_D = builder.THREE_D;
		this.USE_GLOBAL_LOGGER = builder.USE_GLOBAL_LOGGER;
		this.DRAW = builder.DRAW;
		this.DRAW_TRACE = builder.DRAW_TRACE;
		this.DRAW_TRACE_LENGTH = builder.DRAW_TRACE_LENGTH;
        this.DRAW_ROBOT_STROKE_SIZE = builder.DRAW_ROBOT_STROKE_SIZE;
		this.MAX_FPS = builder.MAX_FPS;
		this.DRAWER = builder.DRAWER;
		this.SIM_TIMEOUT = builder.SIM_TIMEOUT;
		this.TIMEOUT = builder.TIMEOUT;
		this.DRAW_WAYPOINTS = builder.DRAW_WAYPOINTS;
		this.DRAW_OBSPOINTS = builder.DRAW_OBSPOINTS;
		this.DRAW_WAYPOINT_NAMES = builder.DRAW_WAYPOINT_NAMES;
		this.DRAW_ROBOT_TYPE = builder.DRAW_ROBOT_TYPE;
		
	}
}