package edu.illinois.mitra.demo.GroupTag;

import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
		settings.BOTS("Model_iRobot").COUNT = 3;
		settings.GPS_POSITION_NOISE(4);
		settings.TIC_TIME_RATE(4);
        settings.WAYPOINT_FILE("grouptagapp/waypoints/tag.wpt");
        settings.INITIAL_POSITIONS_FILE("grouptagapp/waypoints/start.wpt");
        settings.DRAW_TRACE_LENGTH(-1);
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new TagDrawer());
		settings.DRAW__ROBOT_TYPE(true);

		Simulation sim = new Simulation(CelebrityChaserApp.class, settings.build());
		sim.start();
	}

}
