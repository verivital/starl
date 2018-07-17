package edu.illinois.mitra.demo.race;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
		settings.OBSPOINT_FILE("raceApp/waypoints/Obstacles.wpt");
		settings.OBSPOINT_FILE("");
		settings.BOTS(Model_GhostAerial.class.getSimpleName()).COUNT = 1;
		settings.GPS_POSITION_NOISE(4);
		settings.TIC_TIME_RATE(1);
        settings.WAYPOINT_FILE("raceApp/waypoints/four.wpt");
        settings.INITIAL_POSITIONS_FILE("raceApp/waypoints/start.wpt");
        settings.DRAW_TRACE_LENGTH(-1);
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new RaceDrawer());
		settings.DRAW_TRACE(true);
		settings.DRAW__ROBOT_TYPE(true);

		Simulation sim = new Simulation(RaceApp.class, settings.build());
		sim.start();
	}

}
