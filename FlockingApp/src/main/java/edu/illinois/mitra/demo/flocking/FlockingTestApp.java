package edu.illinois.mitra.demo.flocking;


import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.motion.MotionParameters;
import edu.illinois.mitra.starl.motion.RobotMotion;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PositionList;

public class FlockingTestApp extends LogicThread {

    private enum STAGE { START, MOVE, DONE }
    private STAGE stage = STAGE.START;

    private int n_waypoints;
    private int cur_waypoint = 0;
    PositionList<ItemPosition> destinations = new PositionList();
    String wpn = "wp";

    public FlockingTestApp(GlobalVarHolder gvh) {
        super(gvh);

        MotionParameters.Builder settings = new MotionParameters.Builder();
        settings = settings.ENABLE_ARCING(true);
        settings = settings.STOP_AT_DESTINATION(true);
        settings = settings.COLAVOID_MODE(MotionParameters.COLAVOID_MODE_TYPE.USE_COLAVOID); // buggy, just goes back, deadlocks...
        MotionParameters param = settings.build();
        gvh.plat.moat.setParameters(param);

        //n_waypoints = gvh.gps.getWaypointPositions().getNumPositions();
        n_waypoints = Integer.MAX_VALUE;
        String n = wpn + gvh.id.getName() + cur_waypoint;
        destinations.update(new ItemPosition(n, 500, 500, 0));
    }

    private double a_ij(double[] x_i, double[] x_j, double r_sig, double h, double epsilon) {
        double[] subij = x_i.clone();
        for (int i = 0; i < x_i.length; i++)
        {
            subij[i] -= x_j[i];
        }
        return rho_h( (sig_norm(subij, epsilon)/r_sig), h);
    }

    private double norm(double[] a) {
        double d = 0;
        for (int i = 0; i < a.length; i++) {
            d += Math.pow(a[i], 2);
        }
        return Math.sqrt(d);
    }


    private double phi(double z, double a, double b)
    {
        // 0 < a <= b
        double c = Math.abs(a - b)/Math.sqrt(4*a*b);
        // => phi(0, a, b) = 0

        return (1/2)*(((a + b) * sigma_1(z + c)) + (a - b));
    }

    private double phi_a(double z, double r_sig, double d_sig, double h, double a, double b)
    {
        return rho_h(z / r_sig, h) * phi(z - d_sig, a, b);
    }

    private double psi(double z)
    {
        return Math.pow(z, 2);
    }

    private double rho_h(double z, double h) {
        //rho_h(z) = 1                                    if z \in [0,h)
        //           1/2 * (1 + cos(pi * (z - h)/1 -h))   if z \in [h,1]
        //			 0                                    else
        //if (z >= 0 && z < h)
        //return 1;
        //elseif (z >= h && z <= 1)
        //return (1/2) * (1 + cos(pi * (z - h)/1 -h));
        // else
        //	return 0;

        // new: not smooth, determined in mathematica
        if (z >= 0 && z < 1)
            return 1 - z;
        else
            return 0;

        //return norm(outa - out);
    }

    private double sig_norm(double[] z, double epsilon) {
        return norm(z);
        //out = (1 / epsilon) * (sqrt(1 + epsilon * (norm(z,2))^2)-1);
    }

    private double sigma_1(double z) {
        return ((z)/Math.sqrt(1 + Math.pow(z,2)));
    }

    private double[] n_ij(double[] x_i, double[] x_j, double epsilon) {
        double[] subji = x_j.clone();
        for (int i = 0; i < x_i.length; i++) {
            subji[i] -= x_i[i];
        }
        // relies on first computing subji
        for (int i = 0; i < x_i.length; i++) {
            subji[i] /= Math.sqrt(1 + (epsilon * (Math.pow(norm(subji), 2))));
        }
        return subji;
    }

    @Override
    public List<Object> callStarL() {
        String robotName = gvh.id.getName();
        Integer robotNum = gvh.id.getIdNumber();

        while(true) {

            double r_comm = 25.0;
            double r_comm_sig;
            double r_lattice = 12.0;
            double r_lattice_sig;
            double r_safety = 12.0;
            double r_safety_sig;

            double epsilon = 0.1;
            double a=5;
            double b=5;
            double ha=0.2;
            double hb=0.9;

            double c1gamma=0.1;
            double c2gamma=0.1;
            double c1beta=0.25;
            double c2beta=0.1;

            double r_lattice_prime = 0.6 * r_lattice;
            double r_comm_prime = 1.2 * r_lattice_prime;

            r_comm_sig    = sig_norm(new double[] {r_comm}, epsilon);
            r_lattice_sig = sig_norm(new double[] {r_lattice}, epsilon);
            r_safety_sig  = sig_norm(new double[] {r_safety}, epsilon);


            switch (stage) {
                case START:
                {
                    //gvh.trace.traceStart();
                    stage = STAGE.MOVE;
                    System.out.println(robotName + ": Starting motion!");
                    break;
                }

                case MOVE:
                {
                    //if(!gvh.plat.moat.inMotion) {
                    //if(cur_waypoint < n_waypoints) {
                    //System.out.println(robotName + ": I've stopped moving!");

                    cur_waypoint ++;
                    String n = wpn + gvh.id.getName() + cur_waypoint;

                    // circle formation
                    int x = 0, y = 0, theta = 0;
                    int rc = 2000000;
                    int num_nbrs = 0;

                    PositionList<ItemPosition> plAll = gvh.gps.get_robot_Positions(); // all positions

                    //ItemPosition myRp = plAll.getPosition(robotName); // my position
                    ItemPosition myRp =gvh.gps.getMyPosition();

                    double[] u = {0.0, 0.0};

                    // get "neighbor" positions, defined as any other robot within rc distance
                    for (ItemPosition rp : plAll.getList()) {
                        // don't use my position
                        //if (rp.getName() == robotName) {
                        if (rp == myRp) {
                            continue;
                        }

                        if (myRp.distanceTo(rp) < rc) {
                            x += rp.getX();
                            y += rp.getY();
                            theta += ((Model_Ground)rp).angle;
                            num_nbrs++;


                            double[] xj = {rp.getX(), rp.getY()};
                            double[] xi = {myRp.getX(), myRp.getY()};


                            double[] subji = xj;
                            //double[] vsubji = ;
                            double[] vsubji = {0.0, 0.0};
                            double[] vj = {0.0, 0.0};
                            double[] vi = {0.0, 0.0};

                            for (int idx = 0; idx < xj.length; idx++) {
                                subji[idx] -= xi[idx];
                                vsubji[idx] = vj[idx] - vi[idx];
                            }


                            double[] utmp =  n_ij(xi, xj, epsilon);

                            for (int idx = 0; idx < u.length; idx++) {
                                double tmp = sig_norm (subji, epsilon );
                                tmp = phi_a(tmp, r_comm_sig, r_lattice_sig, ha, a, b); // gradient
                                utmp[idx] *= tmp;
                                u[idx] += utmp[idx];
                                u[idx] += (a_ij(xi, xj, r_comm_sig, ha, epsilon) * ((vsubji[idx]))); // consensus
                                u[idx] += -c1gamma*((xi[idx] - xj[idx])) - c2gamma*(vi[idx] - vj[idx]); // i vs j...
                            }

                            //break;
                        }

                    }
                    int N = plAll.getNumPositions();
                    int r = 170; // ~30mm
                    x /= num_nbrs;
                    y /= num_nbrs; // x and y define the centroid of a circle with radius N*r
                    theta /= num_nbrs;

                    //double m = (robotNum % 3 == 0) ? 0.33 : 1; // todo: concentric circles?
                    double m = 1.0;

                    /*x += num_nbrs*m*r*Math.sin(robotNum);
                    y += num_nbrs*m*r*Math.cos(robotNum);*/
                    //destinations.update(new ItemPosition(n, robotNum * 100, 100 * ((robotNum % 2 == 0) ? 0 : 1), 0));

                    System.out.println(robotName + ": Controller values (" + u[0] + ", " + u[1] + ")!");


                    /*x = myRp.getX() + (int)u[0];
                    y = myRp.getY() + (int)u[1];*/

                    // acceleration and velocity

                    //frv = fr(qr, pr); //...fr returns zeros,,,
                    //if constrain ~= 0
                    //    frvs = sign(frv).*(min(abs(frv),a_max));
                    //else
                    //    frvs=frv;
                    //end
                    //qr=qr + pr.*(tcyc/tdiv) + frvs.*((tcyc/tdiv)^2);
                    //pr=pr + frvs.*(tcyc/tdiv);


                    destinations.update(new ItemPosition(n, x, y, theta));
                    System.out.println(destinations.getPosition(n).toString() + " destination");
                    gvh.plat.moat.goTo(destinations.getPosition(n));


                    //olfati-saber controller
                    //uGradient(i,:) = uGradient(i,:) + phi_a(sig_norm ( q_delay(js(j),:) - q_delay(i,:), epsilon ), r_comm_sig, r_lattice_sig, ha, a, b) * n_ij(q_delay(i,:), q_delay(js(j),:), epsilon);
                    //uConsensus(i,:) = uConsensus(i,:) + (a_ij(q_delay(i,:), q_delay(js(j),:), r_comm_sig, ha, epsilon) * ((p_delay(js(j),:) - p_delay(i,:))));
                    //uGamma(i,:) = -c1gamma*((q_delay(i,:) - qr(i,:))) - c2gamma*(p_delay(i,:) - pr(i,:));
                    //u(i,:) = uGradient(i,:) + uConsensus(i,:) + uGamma(i,:);

                    System.out.println(robotName + ": New destination is (" + destinations.getPosition(n).getX() + ", " + destinations.getPosition(n).getY() + ")!");
                    System.out.println(robotName + ": Current posn is (" + myRp.getX() + ", " + myRp.getY() + ")!");


                    // todo: after circle formation, calculate new position based on positions of nearest left and right neighbor (on the circle)
                    //} else {
                    //	stage = STAGE.DONE;
                    //}
                    //}
                    // wait here while robot is in motion
                    while (gvh.plat.moat.inMotion) {
                        gvh.sleep(100);
                    }

                    stage = STAGE.START; // repeat
                    break;


                }

                case DONE:
                    gvh.trace.traceEnd();
                    return Arrays.asList(results);
            }

            gvh.sleep(100);
        }
    }
}