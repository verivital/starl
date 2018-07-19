package edu.illinois.mitra.demo.follow;

import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
		settings.BOTS("Model_iRobot").COUNT = 5;
		settings.TIC_TIME_RATE(5);
        settings.WAYPOINT_FILE("followApp/waypoints/five.wpt");		//must specify relative path
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new FollowDrawer());
		
		Simulation sim = new Simulation(FollowApp.class, settings.build());
		sim.start();
	}

}
