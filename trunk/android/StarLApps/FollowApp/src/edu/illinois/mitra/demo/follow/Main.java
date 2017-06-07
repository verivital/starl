package edu.illinois.mitra.demo.follow;

import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
        settings.N_IROBOTS(3);
		settings.TIC_TIME_RATE(2);
        settings.WAYPOINT_FILE("more.wpt");
		//settings.WAYPOINT_FILE(System.getProperty("user.dir")+"\\trunk\\android\\StarLApps\\FollowApp\\waypoints\\four1.wpt");
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new FollowDrawer());
		
		Simulation sim = new Simulation(FollowApp.class, settings.build());
		sim.start();
	}

}
