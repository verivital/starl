package edu.illinois.mitra.demo.circle;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import edu.illinois.mitra.starl.functions.PickedLeaderElection;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.motion.MotionParameters;
import edu.illinois.mitra.starl.objects.ItemPosition;

import edu.illinois.mitra.starl.functions.BarrierSynchronizer;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.Synchronizer;
import edu.illinois.mitra.starl.objects.PositionList;

//TODO: Cleanup math.

public class CircleApp extends LogicThread {

    private static final boolean RANDOM_DESTINATION = false;
    private String wpn = "wp";
    private int cur_waypoint = 0;
    private int n_waypoints;

    PositionList<ItemPosition> destinations = new PositionList<>();

    private enum STAGE { START, SYNC, ELECT, MOVE, DONE }
    private STAGE stage = STAGE.START;

    private LeaderElection le;
    private Synchronizer sn;

	public CircleApp(GlobalVarHolder gvh) {
        super(gvh);
        //Common.MESSAGE_TIMING = Common.MessageTiming.MSG_ORDERING_LAMPORT;
        //gvh.trace.traceStart();

        MotionParameters.Builder settings = new MotionParameters.Builder();
        settings = settings.ENABLE_ARCING(true);
        settings = settings.STOP_AT_DESTINATION(true);
        //settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.BUMPERCARS); // buggy, just goes through...
        //settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLBACK); // buggy, just goes back, deadlocks...
        settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID); // buggy, just goes back, deadlocks...

        MotionParameters param = settings.build();
        gvh.plat.moat.setParameters(param);

        le = new RandomLeaderElection(gvh);
        sn = new BarrierSynchronizer(gvh);

        //n_waypoints = gvh.gps.getWaypointPositions().getNumPositions();
        //n_waypoints = Integer.MAX_VALUE;

	}

	@Override
	public List<Object> callStarL() {
        String robotName = gvh.id.getName();
        Integer robotNum = gvh.id.getIdNumber();
        Integer count = 0;
        Integer leaderNum = 1;

        while(true) {
            switch (stage) {
                case START: {
                    sn.barrierSync("round" + count.toString());
                    stage = STAGE.SYNC;

                    System.out.printf("robot %d, round " + count.toString() + "\n", robotNum);

                    //gvh.trace.traceStart();
                    //stage = STAGE.MOVE;
                    //gvh.plat.moat.goTo(gvh.gps.getWaypointPosition("DEST"+cur_waypoint));
                    //System.out.println(robotName + ": Starting motion!");
                    break;
                }
                case SYNC: {
                    if (sn.barrierProceed("round" + count.toString())) {
                        stage = STAGE.ELECT;
                        le.elect();
                    }
                    break;
                }
                case ELECT: {
                    if(le.getLeader() != null) {
                        System.out.printf("robot %d, leader is: " + le.getLeader() + "\n", robotNum);
                        leaderNum = le.getLeaderID();
                        stage = STAGE.MOVE;
                    }
                    break;
                }
                case MOVE: {
                    if(!gvh.plat.moat.inMotion) {
                        //if(cur_waypoint < n_waypoints) { }
                        //System.out.println(robotName + ": I've stopped moving!");
                        //String name = wpn + gvh.id.getName() + cur_waypoint;
                        //System.out.println(robotName + ": New destination is (" + destinations.getPosition(name).getX() + ", " + destinations.getPosition(name).getY() + ")!");

                        cur_waypoint ++;
                        name = wpn + gvh.id.getName() + cur_waypoint;

                        // circle formation
                        int x = 0, y = 0, theta = 0;

                        PositionList<ItemPosition> plAll = gvh.gps.get_robot_Positions();
                        int N = plAll.getNumPositions();
                        for (ItemPosition rp : plAll.getList()) {
                            x += rp.getX();
                            y += rp.getY();

                            if(rp instanceof Model_Ground)
                            theta += ((Model_Ground) rp).getAngle();
                        }

                        int r = 130; // ~30mm
                        x = x / N;
                        y = y / N; // x and y define the centroid of a circle with radius N*r
                        theta /= N;

                        //double m = (robotNum % 3 == 0) ? 0.33 : 1; // TODO: concentric circles?
                        double m = 1.0;

                        x += N*m*r*Math.sin(robotNum);
                        y += N*m*r*Math.cos(robotNum);
                       /* destinations.update(new ItemPosition(name, robotNum * 100, 100 * ((robotNum % 2 == 0) ? 0 : 1), 0));

                        ItemPosition dest = new ItemPosition(name, x, y, theta);*/

                        //int offset = (int)Math.sqrt(N)* count; // default is i-1
                        int dir = leaderNum % 2 == 0 ? -1 : 1; // ccw vs. cw based on odd/even leader number
                        //int offset = dir * (int) (Math.min( Math.floor(Math.sqrt(N)) - 1, leaderNum) * count);
                        int offset = dir * 1;
                        count += 1;
                        ItemPosition dest;
                        if (count % 2 == 0) {
                            double rnx = N * 2;
                            double rny = N * 2;
                            dest = new ItemPosition(name, (int) rnx * (int)Math.toDegrees(Math.cos(2*Math.PI * (robotNum+offset) / N)), (int) rny *(int)Math.toDegrees(Math.sin(2*Math.PI * (robotNum+offset) / N)), 1);
                        }
                        else
                        {
                            double tmpx = 2*Math.PI * (robotNum+offset) / N;
                            double tmpy = 2*Math.PI * (robotNum+offset) / N;
                            double rnx = N * 2;
                            double rny = N * 2;
                            dest = new ItemPosition(name, (int)rnx * (int)Math.toDegrees(Math.cos(2*Math.PI * (robotNum+offset) / N)), (int)rny *(int)Math.toDegrees(Math.sin(2*Math.PI * (robotNum+offset) / N)), 1);
                        }
                        //offset = 0;
                        //double tmpx = 2*Math.PI * (robotNum+offset) / N;
                        //double tmpy = 2*Math.PI * (robotNum+offset) / N;

                        //dest = new ItemPosition(n, N * 4 * , N * 4 *(int)Math.toDegrees(Math.sin(2*Math.PI * (robotNum+offset) / N)), 1);
                        //dest = new ItemPosition(n, N * 4 * (int)Math.toDegrees(Math.cos(2*Math.PI * (robotNum+offset) / N)), N * 4 *(int)Math.toDegrees(Math.sin(2*Math.PI * (robotNum+offset) / N)), 1);
                        //x = 16*(Math.sin(t).^3);
                        //y = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
                        //tmpx = 16*Math.sin(Math.pow(tmpx, 3));
                        //tmpy = 13*Math.cos(tmpy) - 5*Math.cos(2*tmpy) - 2*Math.cos(3*tmpy) - Math.cos(4*tmpy);
                        //dest = new ItemPosition(n, N*(int)Math.toDegrees(tmpx), N*(int)Math.toDegrees(tmpy), 0);

                        //tmpx = 16*Math.toDegrees(Math.sin(Math.pow(tmpx, 3)));
                        //tmpy = 13*Math.toDegrees(Math.cos(tmpy)) - 5*Math.toDegrees(Math.cos(2*tmpy)) - 2*Math.toDegrees(Math.cos(3*tmpy)) - Math.toDegrees(Math.cos(4*tmpy));
                        //dest = new ItemPosition(n, N/2*(int)tmpx, N/2*(int)tmpy, 0);

                        destinations.update(dest);
                        //gvh.plat.moat.goTo(destinations.getPosition(name));
                        gvh.plat.moat.goTo(dest);



                        // TODO: after circle formation, calculate new position based on positions of nearest left and right neighbor (on the circle)
                        //} else {
                        //        stage = STAGE.DONE;
                        //}
                    }

                    // wait here while robot is in motion
                    while (gvh.plat.moat.inMotion) {
                        gvh.sleep(100);
                    }

                    stage = STAGE.START; // repeat

                    break;
                }

                case DONE: {
                    gvh.trace.traceEnd();
                    return Arrays.asList(results);
                }
            }
            gvh.sleep(100);
            //Random rand = new Random();
            //gvh.sleep(100 + rand.nextInt(5)); // weird simulation behavior if things aren't sleep-synchronized
            //gvh.sleep( (robotNum + 1) * 25);
        }
	}

	private static final Random rand = new Random();

	@SuppressWarnings("unchecked")
	private <X, T> T getRandomElement(Map<X, T> map) {
		if(RANDOM_DESTINATION)
			return (T) map.values().toArray()[rand.nextInt(map.size())];
		else
			return (T) map.values().toArray()[0];
	}
}
