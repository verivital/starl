package edu.illinois.mitra.demo.search;

import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		/*This application is to simulate a group of robots searching for something in the room.
		* The object will be sensed if it is close to any robot and no obstacle is between robot and object
		* We will do leader election, assign tasks, perform tasks, Inform others when finished
		* The mutual exclusion on the overlapping path is not our focus in this app.
		* The room coverage algorithm is not our focus in this app.
		*/
		SimSettings.Builder settings = new SimSettings.Builder();
		
		settings.BOTS("Model_iRobot").COUNT = 5;
		settings.N_GOAL_BOTS(0);
		settings.TIC_TIME_RATE(5);
		settings.WAYPOINT_FILE("distributedsearchapp/waypoints/dest.wpt");
		settings.SENSEPOINT_FILE("distributedsearchapp/waypoints/senseObjects.wpt");
		settings.INITIAL_POSITIONS_FILE("distributedsearchapp/waypoints/start.wpt");
		settings.OBSPOINT_FILE("distributedsearchapp/waypoints/Obstacles.wpt");
		
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new DistributedSearchDrawer());
//		settings.MSG_LOSSES_PER_HUNDRED(20);
//		settings.GPS_POSITION_NOISE(-5);
//		settings.GPS_ANGLE_NOISE(1);
//		settings.BOT_RADIUS(400);
		Simulation sim = new Simulation(DistributedSearchApp.class, settings.build());
		sim.start();
	}

}
