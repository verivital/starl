package edu.illinois.mitra.starl.objects;

import java.util.*;

import edu.illinois.mitra.starl.motion.RRTNode;

/**
 * The obstacle is defined here
 * Each obstacle is a polygon, the list of points should construct a closed shape
 *
 * @author Yixiao Lin, updated by Tim Liang and Stirling Carter
 * @version 2.0
 */
public class Obstacles {
    public Vector<Point3d> obstacle;
    public int height;
    public long timeFrame;
    public boolean hidden;
    public boolean grided;
    //time that the obstacle will stay in the system, in milliseconds
    //if -1, it is a static obstacle
    //once zero, it will be removed from the obsList

    public Obstacles() {
        obstacle = new Vector<Point3d>(4, 3);
    }

    public Obstacles(Vector<Point3d> obstacle1) {
        obstacle = obstacle1;
    }

    public Obstacles(Obstacles original) {
        obstacle = new Vector<Point3d>(4, 3);
        for (int i = 0; i < original.obstacle.size(); i++) {
            add(original.obstacle.get(i).getX(), original.obstacle.get(i).getY());
        }
        grided = original.grided;
        timeFrame = original.timeFrame;
        hidden = original.hidden;
        height = -1;
    }

    //method for adding unknown obstacles
    public Obstacles(int x, int y) {
        obstacle = new Vector<Point3d>(4, 3);
        add(x, y, 0);
        grided = false;
        timeFrame = -1;
        height = -1;
    }

    public Obstacles(int x, int y, int z) {
        obstacle = new Vector<Point3d>(4, 3);
        add(x, y, z);
        grided = false;
        timeFrame = -1;
        height = -1;
    }

    public void add(int x, int y) {
        add(x, y, 0);
    }

    public void add(int x, int y, int z) {
        Point3d temp = new Point3d(x, y, z);
        obstacle.add(temp);
    }

    /**
     * return a clone so the obstacles cannot be modified
     * TODO: check that this deep copies all the points too
     *
     * @return
     */
    public Vector<Point3d> getObstacleVector() {
        return (Vector<Point3d>) this.obstacle.clone();
    }

    /**
     * check if line from current to destination has intersection with any part of the object
     * return true if cross
     *
     * @param destination
     * @param current
     * @return
     */
    public boolean checkCross(ItemPosition destination, ItemPosition current) {
        double x1, x2, x3, x4, y1, y2, y3, y4;
        x1 = destination.getX();
        y1 = destination.getY();
        x2 = current.getX();
        y2 = current.getY();
        for (int i = 0; i < obstacle.size(); i++) {
            for (int j = i + 1; j < obstacle.size() && obstacle.elementAt(i) != null; j++) {
                x3 = obstacle.elementAt(i).getX();
                y3 = obstacle.elementAt(i).getY();
                x4 = obstacle.elementAt(j).getX();
                y4 = obstacle.elementAt(j).getY();
                if (linesIntersect(x1, y1, x2, y2, x3, y3, x4, y4)) {
                    return true;
                }
            }
        }
        return false;

    }

    //line intersection calculation method to replace java.awt.geom.Line2D.intersectsLine() since
    //the java.awt library is not part of android (also this one is supposedly 25% faster)
    //source: http://www.java-gaming.org/index.php?topic=22590.0
    private static boolean linesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        // Return false if either of the lines have zero length
        if (x1 == x2 && y1 == y2 ||
                x3 == x4 && y3 == y4) {
            return false;
        }
        // Fastest method, based on Franklin Antonio's "Faster Line Segment Intersection" topic "in Graphics Gems III" book (http://www.graphicsgems.org/)
        double ax = x2 - x1;
        double ay = y2 - y1;
        double bx = x3 - x4;
        double by = y3 - y4;
        double cx = x1 - x3;
        double cy = y1 - y3;

        double alphaNumerator = by * cx - bx * cy;
        double commonDenominator = ay * bx - ax * by;
        if (commonDenominator > 0) {
            if (alphaNumerator < 0 || alphaNumerator > commonDenominator) {
                return false;
            }
        } else if (commonDenominator < 0) {
            if (alphaNumerator > 0 || alphaNumerator < commonDenominator) {
                return false;
            }
        }
        double betaNumerator = ax * cy - ay * cx;
        if (commonDenominator > 0) {
            if (betaNumerator < 0 || betaNumerator > commonDenominator) {
                return false;
            }
        } else if (commonDenominator < 0) {
            if (betaNumerator > 0 || betaNumerator < commonDenominator) {
                return false;
            }
        }
        if (commonDenominator == 0) {
            // This code wasn't in Franklin Antonio's method. It was added by Keith Woodward.
            // The lines are parallel.
            // Check if they're collinear.
            double y3LessY1 = y3 - y1;
            double collinearityTestForP3 = x1 * (y2 - y3) + x2 * (y3LessY1) + x3 * (y1 - y2);   // see http://mathworld.wolfram.com/Collinear.html
            // If p3 is collinear with p1 and p2 then p4 will also be collinear, since p1-p2 is parallel with p3-p4
            if (collinearityTestForP3 == 0) {
                // The lines are collinear. Now check if they overlap.
                if (x1 >= x3 && x1 <= x4 || x1 <= x3 && x1 >= x4 ||
                        x2 >= x3 && x2 <= x4 || x2 <= x3 && x2 >= x4 ||
                        x3 >= x1 && x3 <= x2 || x3 <= x1 && x3 >= x2) {
                    if (y1 >= y3 && y1 <= y4 || y1 <= y3 && y1 >= y4 ||
                            y2 >= y3 && y2 <= y4 || y2 <= y3 && y2 >= y4 ||
                            y3 >= y1 && y3 <= y2 || y3 <= y1 && y3 >= y2) {
                        return true;
                    }
                }
            }
            return false;
        }
        return true;
    }

    /**
     * check if the itemPosotion destination is reachable by robots
     * return true if robot can reach it
     *
     * @param destination
     * @param radius
     * @return
     */

    public boolean validItemPos(ItemPosition destination, double radius) {
        if (destination == null)
            return false;
        if (obstacle.size() == 0)
            return true;

        Point3d nextpoint = obstacle.firstElement();
        Point3d curpoint = obstacle.firstElement();
        int[] x = new int[obstacle.size()];
        int[] y = new int[obstacle.size()];

        for (int j = 0; j < obstacle.size(); j++) {
            curpoint = obstacle.get(j);
            if (j == obstacle.size() - 1) {
                nextpoint = obstacle.firstElement();
            } else {
                nextpoint = obstacle.get(j + 1);
            }
            double x1 = curpoint.getX();
            double y1 = curpoint.getY();
            double x2 = nextpoint.getX();
            double y2 = nextpoint.getY();
            x[j] = curpoint.getX();
            y[j] = curpoint.getY();
            double px = destination.getX();
            double py = destination.getY();
            if (pointToLineSeg(px, py, x1, y1, x2, y2) < radius) {
                return false;
            }

        }
        Polygon obspoly = new Polygon(x, y, obstacle.size());
        return !obspoly.contains(destination.getX(), destination.getY());
    }

    /**
     * check if the itemPosotion destination is reachable by robots
     * return true if robot can reach it
     *
     * @param destination
     * @return
     */
    public boolean validItemPos(ItemPosition destination) {
        Point3d curpoint = obstacle.firstElement();
        int[] x = new int[obstacle.size()];
        int[] y = new int[obstacle.size()];

        for (int j = 0; j < obstacle.size(); j++) {
            curpoint = obstacle.get(j);
            x[j] = curpoint.getX();
            y[j] = curpoint.getY();

        }
        Polygon obspoly = new Polygon(x, y, obstacle.size());
        return !obspoly.contains(destination.getX(), destination.getY());
    }

    public double findMinDist(RRTNode destNode, RRTNode currentNode) {
        Point3d nextpoint = obstacle.firstElement();
        Point3d curpoint = obstacle.firstElement();
        double minDist = Double.MAX_VALUE;
        double cx1 = destNode.position.getX();
        double cy1 = destNode.position.getY();
        double cx2 = currentNode.position.getX();
        double cy2 = currentNode.position.getY();

        for (int j = 0; j < obstacle.size(); j++) {
            curpoint = obstacle.get(j);
            if (j == obstacle.size() - 1) {
                nextpoint = obstacle.firstElement();
            } else {
                nextpoint = obstacle.get(j + 1);
            }
            double sx1 = curpoint.getX();
            double sy1 = curpoint.getY();
            double sx2 = nextpoint.getX();
            double sy2 = nextpoint.getY();

            double dist1 = shortestDistance(cx1, cy1, sx1, sy1, sx2, sy2);//segment.ptSegDist(current.x1, current.y1);
            double dist2 = shortestDistance(cx2, cy2, sx1, sy1, sx2, sy2);//segment.ptSegDist(current.x2, current.y2);
            double dist3 = shortestDistance(sx1, sy1, cx1, cy1, cx2, cy2);//current.ptSegDist(segment.x1, segment.y1);
            double dist4 = shortestDistance(sx2, sy2, cx1, cy1, cx2, cy2);//current.ptSegDist(segment.x2, segment.y2);
            //System.out.println("dist 1: " + dist1);
            //System.out.print("dist 2: " + dist2);
            //System.out.printf("| %f %f %f %f %f %f \n", cx2, cy2, sx1, sy1, sx2, sy2);
            //System.out.println("dist 3: " + dist3);
            //System.out.println("dist 4: " + dist4);
            double temp1 = Math.min(dist1, dist2);
            double temp2 = Math.min(dist3, dist4);
            double minDistNow = Math.min(temp1, temp2);
            minDist = Math.min(minDistNow, minDist);
        }
        return minDist;
    }

    //returns the shortest distance between a point to a line segment
    //source: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment?page=1&tab=votes#tab-top
    private static double pointToLineSeg(double px, double py, double x1, double y1, double x2, double y2) {
        double A = px - x1;
        double B = py - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = -1;
        if (len_sq != 0) {
            param = dot / len_sq;
        }

        double xx, yy;
        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = px - xx;
        double dy = px - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static double shortestDistance(double x3,double y3, double x1,double y1,double x2,double y2)
    {
        double px=x2-x1;
        double py=y2-y1;
        double temp=(px*px)+(py*py);
        double u=((x3 - x1) * px + (y3 - y1) * py) / (temp);
        if(u>1){
            u=1;
        }
        else if(u<0){
            u=0;
        }
        double x = x1 + u * px;
        double y = y1 + u * py;

        double dx = x - x3;
        double dy = y - y3;
        return Math.sqrt(dx*dx + dy*dy);

    }

    public Point3d getClosestPointOnSegment(int sx1, int sy1, int sx2, int sy2, int px, int py) {
        double xDelta = sx2 - sx1;
        double yDelta = sy2 - sy1;
        double u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

        final Point3d closestPoint;
        if (u < 0) {
            closestPoint = new Point3d(sx1, sy1);
        } else if (u > 1) {
            closestPoint = new Point3d(sx2, sy2);
        } else {
            closestPoint = new Point3d((int) Math.round(sx1 + u * xDelta), (int) Math.round(sy1 + u * yDelta));
        }

        return closestPoint;
    }

    /**
     * gridify the map, make the obstacle map grid like. For any Grid that contains obstacle, that grid is considered an obstacle
     *
     * @param a
     */
    public void ToGrid(int a) {
        if (grided) {
            return;
        }
        //System.out.println(obstacle);
        switch (obstacle.size()) {
            case 1:
                Point3d leftBottom1 = new Point3d(obstacle.firstElement().getX() - ((obstacle.firstElement().getX()) % a), obstacle.firstElement().getY() - ((obstacle.firstElement().getY()) % a));
                Point3d rightBottom1 = new Point3d((leftBottom1.getX() + a), leftBottom1.getY());
                Point3d rightTop1 = new Point3d((leftBottom1.getX() + a), (leftBottom1.getY() + a));
                Point3d leftTop1 = new Point3d((leftBottom1.getX()), (leftBottom1.getY() + a));
                obstacle.removeAllElements();
                obstacle.add(leftBottom1);
                obstacle.add(rightBottom1);
                obstacle.add(rightTop1);
                obstacle.add(leftTop1);
                break;
            case 2:
                int min_x = Math.min(obstacle.firstElement().getX(), obstacle.get(1).getX());
                min_x = min_x - (min_x % a);
                int max_x = Math.max(obstacle.firstElement().getX(), obstacle.get(1).getX());
                max_x = max_x - (max_x % a) + a;
                int min_y = Math.min(obstacle.firstElement().getY(), obstacle.get(1).getY());
                min_y = min_y - (min_y % a);
                int max_y = Math.max(obstacle.firstElement().getY(), obstacle.get(1).getY());
                max_y = max_y - (max_y % a) + a;

                Point3d leftBottom2 = new Point3d(min_x, min_y);
                Point3d rightBottom2 = new Point3d(max_x, min_y);
                Point3d rightTop2 = new Point3d(max_x, max_y);
                Point3d leftTop2 = new Point3d(min_x, max_y);
                obstacle.removeAllElements();
                obstacle.add(leftBottom2);
                obstacle.add(rightBottom2);
                obstacle.add(rightTop2);
                obstacle.add(leftTop2);
                break;
            case 4:
                int min_x3 = Math.min(obstacle.firstElement().getX(), obstacle.get(1).getX());
                min_x3 = Math.min(min_x3, obstacle.get(2).getX());
                min_x3 = Math.min(min_x3, obstacle.get(3).getX());
                min_x3 = min_x3 - (min_x3 % a);

                int max_x3 = Math.max(obstacle.firstElement().getX(), obstacle.get(1).getX());
                max_x3 = Math.max(max_x3, obstacle.get(2).getX());
                max_x3 = Math.max(max_x3, obstacle.get(3).getX());
                max_x3 = max_x3 - (max_x3 % a) + a;

                int min_y3 = Math.min(obstacle.firstElement().getY(), obstacle.get(1).getY());
                min_y3 = Math.min(min_y3, obstacle.get(2).getY());
                min_y3 = Math.min(min_y3, obstacle.get(3).getY());
                min_y3 = min_y3 - (min_y3 % a);

                int max_y3 = Math.max(obstacle.firstElement().getY(), obstacle.get(1).getY());
                max_y3 = Math.max(max_y3, obstacle.get(2).getY());
                max_y3 = Math.max(max_y3, obstacle.get(3).getY());
                max_y3 = max_y3 - (max_y3 % a) + a;

                Point3d leftBottom3 = new Point3d(min_x3, min_y3);
                Point3d rightBottom3 = new Point3d(max_x3, min_y3);
                Point3d rightTop3 = new Point3d(max_x3, max_y3);
                Point3d leftTop3 = new Point3d(min_x3, max_y3);
                obstacle.removeAllElements();
                obstacle.add(leftBottom3);
                obstacle.add(rightBottom3);
                obstacle.add(rightTop3);
                obstacle.add(leftTop3);
                break;
            default:
                System.out.println("not an acceptable demension of " + obstacle.size() + " to be grided");
                break;
        }
        grided = true;

    }
}