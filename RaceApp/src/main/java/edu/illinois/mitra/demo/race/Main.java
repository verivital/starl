package edu.illinois.mitra.demo.race;

import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

	public static void main(String[] args) {
		SimSettings.Builder settings = new SimSettings.Builder();
		settings.OBSPOINT_FILE("raceApp/waypoints/Obstacles.wpt");
		settings.N_o3DR(0);
		settings.OBSPOINT_FILE("");
		settings.N_IROBOTS(0);
		settings.N_QUADCOPTERS(0);
		settings.N_GHOSTS(1);
		settings.N_MAVICS(0);
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
