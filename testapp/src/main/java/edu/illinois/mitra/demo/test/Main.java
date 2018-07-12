package edu.illinois.mitra.demo.test;

import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class Main {

    public static void main(String[] args) {
        SimSettings.Builder settings = new SimSettings.Builder();

        //Robots
        settings.N_IROBOTS(0);  //works in all increments
        settings.N_QUADCOPTERS(1);  //Works unless multiple crash.
        settings.N_o3DR(0);     //Works
        settings.N_GHOSTS(0);   //Doesn't target correctly
        settings.N_MAVICS(0);   //Doesn't work, points switch without being hit
        settings.N_PHANTOMS(0); //Doesn't move

        //Files
        settings.WAYPOINT_FILE("testapp/waypoints/single.wpt");     //Found in top level waypoints directory, key must be #-iRob, #-quad, or # for others
        //settings.OBSPOINT_FILE("ObstacleCourse.wpt");
        settings.INITIAL_POSITIONS_FILE("testapp/waypoints/start.wpt");

        //Drawing
        settings.DRAW_WAYPOINTS(false);         //has to be false because drawer class handles waypoints, not simulation class
        settings.DRAWER(new TestDrawer());
        settings.DRAW_TRACE(true);

        //Misc
        settings.TIC_TIME_RATE(1);

        Simulation sim = new Simulation(TestApp.class, settings.build());
        sim.start();
    }

}
