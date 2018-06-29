package edu.illinois.mitra.demo.follow;

import edu.illinois.mitra.starlSim.main.SimSettings;
import edu.illinois.mitra.starlSim.main.Simulation;

public class TestMain {

    public static void main(String[] args) {
        SimSettings.Builder settings = new SimSettings.Builder();

        //Robots
        settings.N_IROBOTS(0);  //works in all increments
        settings.N_QUADCOPTERS(0);  //Works unless multiple crash.
        settings.N_o3DR(1);     //Works

        settings.N_GHOSTS(0);   //Doesn't target correctly
        settings.N_MAVICS(0);   //Doesn't work, points switch without being hit
        settings.N_PHANTOMS(0); //Doesn't move

        settings.N_GBOTS(0);    //Doesn't recognize these as robots
        settings.N_DBOTS(0);
        settings.N_RBOTS(0);


        //Files
        settings.WAYPOINT_FILE("RobotSpecific.wpt");     //Found in top level waypoints directory
        //settings.OBSPOINT_FILE("ObstacleCourse.wpt");
        settings.INITIAL_POSITIONS_FILE("start.wpt");

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
