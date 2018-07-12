package edu.illinois.mitra.demo.circle;

import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
		settings.N_IROBOTS(15); // pick N reasonably large (> ~10) for rotations along arcs instead of going across middle always
		settings.TIC_TIME_RATE(1.5);
        settings.WAYPOINT_FILE("four.wpt");
		//settings.WAYPOINT_FILE(System.getProperty("user.dir")+"\\trunk\\android\\RaceApp\\waypoints\\four1.wpt");
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new CircleDrawer());
		
		Simulation sim = new Simulation(CircleApp.class, settings.build());
		sim.start();
	}

}
