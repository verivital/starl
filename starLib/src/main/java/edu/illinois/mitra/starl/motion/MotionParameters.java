package edu.illinois.mitra.starl.motion;

/**
 * A MotionParameters object contains settings describing speeds and options to
 * use in motion.
 * 
 * @author Adam Zimmerman
 * @see RobotMotion
 * @see MotionAutomaton_Drone
 * @see MotionAutomaton_Ground
 */
public class MotionParameters {

	private static final MotionParameters defaultInstance = new MotionParameters.Builder().build();

	public static MotionParameters defaultParameters() {
		return defaultInstance;
	}

	/**
	 * (Degrees per second?) The maximum allowable speed when turning in place
	 */
	public final int TURNSPEED_MAX;
	/**
	 * (Degrees per second?) The minimum speed when turning in place. In place
	 * turns will "ramp down" to this speed when approaching the goal
	 */
	public final int TURNSPEED_MIN;

	/**
	 * (mm/sec?) The maximum speed when traveling in a straight line
	 */
	public final int LINSPEED_MAX;

	/**
	 * (mm/sec?) The minimum speed when traveling in a straight line.
	 */
	public final int LINSPEED_MIN;

	/**
	 * The minimum distance between the center of the robot and the goal
	 * position that must be achieved.
	 */
	public final int GOAL_RADIUS;

	/**
	 * The distance at which the automaton will begin decelerating from
	 * LINSPEED_MAX TO LINSPEED_MIN.
	 */
	public final int SLOWFWD_RADIUS;

	/**
	 * (Degrees) The maximum allowable angle of error between the direction of
	 * motion and the direction to the goal.
	 */
	public final int SMALLTURN_ANGLE;

	/**
	 * (Degrees) The angle at which the automaton will begin decelerating from
	 * TURNSPEED_MAX to TURNSPEED_MIN
	 */
	public final int SLOWTURN_ANGLE;

	/**
	 * (Centimeters?) The radius of the robot, used for collision detection and
	 * avoidance
	 */
	public final int ROBOT_RADIUS;

	/**
	 * (Degrees) When arcing, the maximum angle to goal at which straight motion
	 * will resume.
	 */
	public final int STRAIGHT_ANGLE;

	/**
	 * (Degrees) If, while arcing, the angle to goal exceeds this value, the
	 * automaton will stop arcing and turn in place to face the goal.
	 */
	public final int ARC_EXIT_ANGLE;

	/**
	 * The maximum distance at which arcing motion will be used
	 */
	public final int ARC_RADIUS;

	/**
	 * If false, the robot will continue whatever motion was in progress when
	 * the destination is reached
	 */
	public final boolean STOP_AT_DESTINATION;

	/**
	 * Enable/disable arcing motion
	 */
	public final boolean ENABLE_ARCING;

	/**
	 * (mm/sec?) The maximum speed when arcing
	 */
	public final int ARCSPEED_MAX;

	/**
	 * (Degrees) The maximum angle at which the automaton will begin traveling
	 * in arcs (if arcing is enabled)
	 */
	public final int ARCANGLE_MAX;

	/**
	 * (Milliseconds) The update rate of the automaton
	 */
	public final int AUTOMATON_PERIOD;

	/**
	 * Sets the collision avoidance mode. Options are:<br> USE_COLAVOID - use
	 * the standard collision avoidance algorithm<br> STOP_ON_COLLISION - stop
	 * if a collision is imminent and don't resume until the blocker moves<br>
	 * BUMPERCARS - ignore collision detection and just keep going no matter
	 * what<br>
	 */
	public final COLAVOID_MODE_TYPE COLAVOID_MODE;

	public static enum COLAVOID_MODE_TYPE {
		USE_COLAVOID, USE_COLBACK, STOP_ON_COLLISION, BUMPERCARS
	};

	/**
	 * (Milliseconds) When avoiding collisions, the maximum length of time to
	 * travel forwards for after turning to avoid a robot
	 */
	public int COLLISION_AVOID_STRAIGHTTIME = 1250;
	public int COLLISION_AVOID_BACKTIME = 300;
	public int COLLISION_AVOID_TURNTIME = 5000;
	

	public static class Builder {
		private int TURNSPEED_MAX = 50; // changed from 110
		private int TURNSPEED_MIN = 25;
		private int LINSPEED_MAX = 250;
		private int LINSPEED_MIN = 175;
		private int GOAL_RADIUS = 75;
		private int SLOWFWD_RADIUS = 700;
		private int SMALLTURN_ANGLE = 3;
		private int SLOWTURN_ANGLE = 25;
		private int ROBOT_RADIUS = 165;
		private int STRAIGHT_ANGLE = 6;
		private int ARC_EXIT_ANGLE = 30;
		private int ARC_RADIUS = 700;
		private boolean STOP_AT_DESTINATION = true;
		private boolean ENABLE_ARCING = true;
		private int ARCSPEED_MAX = 200;
		private int ARCANGLE_MAX = 25;
		private int AUTOMATON_PERIOD = 60;
		private COLAVOID_MODE_TYPE COLAVOID_MODE = COLAVOID_MODE_TYPE.STOP_ON_COLLISION;
		private int COLLISION_AVOID_STRAIGHTTIME = 1250;
		private int COLLISION_AVOID_BACKTIME = 800;
		private int COLLISION_AVOID_TURNTIME = 500;
		
		

		public Builder TURNSPEED_MAX(int TURNSPEED_MAX) {
			this.TURNSPEED_MAX = TURNSPEED_MAX;
			return this;
		}

		public Builder TURNSPEED_MIN(int TURNSPEED_MIN) {
			this.TURNSPEED_MIN = TURNSPEED_MIN;
			return this;
		}

		public Builder LINSPEED_MAX(int LINSPEED_MAX) {
			this.LINSPEED_MAX = LINSPEED_MAX;
			return this;
		}

		public Builder LINSPEED_MIN(int LINSPEED_MIN) {
			this.LINSPEED_MIN = LINSPEED_MIN;
			return this;
		}

		public Builder GOAL_RADIUS(int GOAL_RADIUS) {
			this.GOAL_RADIUS = GOAL_RADIUS;
			return this;
		}

		public Builder SLOWFWD_RADIUS(int SLOWFWD_RADIUS) {
			this.SLOWFWD_RADIUS = SLOWFWD_RADIUS;
			return this;
		}

		public Builder SMALLTURN_ANGLE(int SMALLTURN_ANGLE) {
			this.SMALLTURN_ANGLE = SMALLTURN_ANGLE;
			return this;
		}

		public Builder SLOWTURN_ANGLE(int SLOWTURN_ANGLE) {
			this.SLOWTURN_ANGLE = SLOWTURN_ANGLE;
			return this;
		}

		public Builder ROBOT_RADIUS(int ROBOT_RADIUS) {
			this.ROBOT_RADIUS = ROBOT_RADIUS;
			return this;
		}

		public Builder STRAIGHT_ANGLE(int STRAIGHT_ANGLE) {
			this.STRAIGHT_ANGLE = STRAIGHT_ANGLE;
			return this;
		}

		public Builder ARC_EXIT_ANGLE(int ARC_EXIT_ANGLE) {
			this.ARC_EXIT_ANGLE = ARC_EXIT_ANGLE;
			return this;
		}

		public Builder ARC_RADIUS(int ARC_RADIUS) {
			this.ARC_RADIUS = ARC_RADIUS;
			return this;
		}

		public Builder STOP_AT_DESTINATION(boolean STOP_AT_DESTINATION) {
			this.STOP_AT_DESTINATION = STOP_AT_DESTINATION;
			return this;
		}

		public Builder ENABLE_ARCING(boolean ENABLE_ARCING) {
			this.ENABLE_ARCING = ENABLE_ARCING;
			return this;
		}

		public Builder ARCSPEED_MAX(int ARCSPEED_MAX) {
			this.ARCSPEED_MAX = ARCSPEED_MAX;
			return this;
		}

		public Builder ARCANGLE_MAX(int ARCANGLE_MAX) {
			this.ARCANGLE_MAX = ARCANGLE_MAX;
			return this;
		}

		public Builder AUTOMATON_PERIOD(int AUTOMATON_PERIOD) {
			this.AUTOMATON_PERIOD = AUTOMATON_PERIOD;
			return this;
		}

		public Builder COLAVOID_MODE(COLAVOID_MODE_TYPE COLAVOID_MODE) {
			this.COLAVOID_MODE = COLAVOID_MODE;
			return this;
		}

		public Builder COLLISION_AVOID_STRAIGHTTIME(int COLLISION_AVOID_STRAIGHTTIME) {
			this.COLLISION_AVOID_STRAIGHTTIME = COLLISION_AVOID_STRAIGHTTIME;
			return this;
		}
		
		public Builder COLLISION_AVOID_BACKTIME(int COLLISION_AVOID_BACKTIME) {
			this.COLLISION_AVOID_BACKTIME = COLLISION_AVOID_BACKTIME;
			return this;
		}
		
		public Builder COLLISION_AVOID_TURNTIME(int COLLISION_AVOID_TURNTIME) {
			this.COLLISION_AVOID_TURNTIME = COLLISION_AVOID_TURNTIME;
			return this;
		}

		public MotionParameters build() {
			return new MotionParameters(this);
		}
	}

	private MotionParameters(Builder builder) {
		this.TURNSPEED_MAX = builder.TURNSPEED_MAX;
		this.TURNSPEED_MIN = builder.TURNSPEED_MIN;
		this.LINSPEED_MAX = builder.LINSPEED_MAX;
		this.LINSPEED_MIN = builder.LINSPEED_MIN;
		this.GOAL_RADIUS = builder.GOAL_RADIUS;
		this.SLOWFWD_RADIUS = builder.SLOWFWD_RADIUS;
		this.SMALLTURN_ANGLE = builder.SMALLTURN_ANGLE;
		this.SLOWTURN_ANGLE = builder.SLOWTURN_ANGLE;
		this.ROBOT_RADIUS = builder.ROBOT_RADIUS;
		this.STRAIGHT_ANGLE = builder.STRAIGHT_ANGLE;
		this.ARC_EXIT_ANGLE = builder.ARC_EXIT_ANGLE;
		this.ARC_RADIUS = builder.ARC_RADIUS;
		this.STOP_AT_DESTINATION = builder.STOP_AT_DESTINATION;
		this.ENABLE_ARCING = builder.ENABLE_ARCING;
		this.ARCSPEED_MAX = builder.ARCSPEED_MAX;
		this.ARCANGLE_MAX = builder.ARCANGLE_MAX;
		this.AUTOMATON_PERIOD = builder.AUTOMATON_PERIOD;
		this.COLAVOID_MODE = builder.COLAVOID_MODE;
		this.COLLISION_AVOID_STRAIGHTTIME = builder.COLLISION_AVOID_STRAIGHTTIME;
		this.COLLISION_AVOID_BACKTIME = builder.COLLISION_AVOID_BACKTIME;
		this.COLLISION_AVOID_TURNTIME = builder.COLLISION_AVOID_TURNTIME;
		
	}
}
