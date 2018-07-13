package edu.illinois.mitra.demo.GroupTag;

import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
		settings.N_IROBOTS(5);
		settings.GPS_POSITION_NOISE(4);
		settings.TIC_TIME_RATE(1);
        settings.WAYPOINT_FILE("grouptagapp/waypoints/tag.wpt");
 //       settings.INITIAL_POSITIONS_FILE("raceApp/waypoints/start.wpt");
        settings.DRAW_TRACE_LENGTH(-1);
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new TagDrawer());
		settings.DRAW__ROBOT_TYPE(true);

		Simulation sim = new Simulation(GroupTagApp.class, settings.build());
		sim.start();
	}

}
