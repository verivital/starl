package edu.illinois.mitra.demo.flocking;

import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;



public class Main {



    public static void main(String[] args) {

        SimSettings.Builder settings = new SimSettings.Builder();
		// pick N reasonably large (> ~10) for rotations along arcs instead of going across middle always
		settings.BOTS("Model_iRobot").COUNT = Common.numOFbots;
		settings.TIC_TIME_RATE(5);
        settings.WAYPOINT_FILE("flockingapp/waypoints/four.wpt");		//must specify relative directory
		//settings.WAYPOINT_FILE(System.getProperty("user.dir")+"\\trunk\\android\\RaceApp\\waypoints\\four1.wpt");
		settings.DRAW_WAYPOINTS(false);
		settings.DRAW_WAYPOINT_NAMES(false);
		settings.DRAWER(new FlockingDrawer());
		
		Simulation sim = new Simulation(FlockingApp.class, settings.build());
		sim.start();
	}

}
