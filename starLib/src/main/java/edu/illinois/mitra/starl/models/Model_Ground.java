package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.motion.BTI;
import edu.illinois.mitra.starl.motion.GroundBTI;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starl.objects.PositionList;

public abstract class Model_Ground extends Model {
    public double angle = 0;
    public Type type = Type.GET_TO_GOAL;
    public boolean leftBump = false;
    public boolean rightBump = false;
    public double vFwd = 0;
    public double vRad = 0;
    private int x_p = 0;
    private int y_p = 0;
    private double angle_p = 0;

    public Model_Ground() {}

    public Model_Ground(String name, int x, int y) {
        super(name, x, y);
    }

    public Model_Ground(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    public Model_Ground(ItemPosition t_pos) {
        super(t_pos);
        this.angle = t_pos.index;
    }

    public abstract Class<? extends GroundBTI> getBluetoothInterface();

    @Override
    public String toString() {
        return name + ": " + getPos() + ", angle " + angle;
    }

    /**
     * isFacing
     * @return true if one robot is facing another robot/point
     */
    public boolean isFacing(ItemPosition other) {
        return isFacing(other.getPos());
    }

    public boolean isFacing(Point3i other) {
        if(other == null) {
            return false;
        }
        if(other.getX() == this.getX() && other.getY() == this.getY()) {
            return true;
        }
        double angleT = Math.toDegrees(Math.atan2(other.getY() - this.getY(), other.getX() - this.getX()));
        if(angleT  == 90) {
            if(this.getY() < other.getY())
                angleT = angleT + 90;
            double temp = Common.angleWrap(this.angle);
            return temp > 0;
        }
        if(angleT < 0) {
            angleT += 360;
        }
        double angleT1, angleT2, angleself;
        angleT1 = Common.angleWrap(angleT - 90);
        if(angleT1 < 0) {
            angleT1 += 360;
        }
        angleT2 = Common.angleWrap(angleT + 90);
        if(angleT2 < 0) {
            angleT2 += 360;
        }
        angleself = Common.angleWrap(this.angle);
        if(angleself < 0) {
            angleself += 360;
        }
        if(angleT2 <= 180) {
            return !(angleself < angleT1 && angleself > angleT2);
        } else {
            return !(angleself > angleT2 || angleself < angleT1);
        }
    }

    /**
     * @param other The ItemPosition to measure against
     * @return Number of degrees this position must rotate to face position other
     */
    public int angleTo(ItemPosition other) {
        return angleTo(other.getPos());
    }

    public double getAngle(){
        return angle;
    }

    public int angleTo(Point3i other) {
        if(other == null) {
            return 0;
        }

        int delta_x = other.getX() - this.getX();
        int delta_y = other.getY() - this.getY();
        double angle = this.angle;
        int otherAngle = (int) Math.toDegrees(Math.atan2(delta_y,delta_x));
        if(angle > 180) {
            angle -= 360;
        }
        int retAngle = Common.min_magitude((int)(otherAngle - angle),(int)(angle - otherAngle));
        retAngle = (int)Common.angleWrap(retAngle);
        if(retAngle > 180) {
            retAngle = retAngle-360;
        }
        if(retAngle <= -180) {
            retAngle = retAngle+360;
        }
        if(retAngle > 180 || retAngle< -180){
            System.out.println(retAngle);
        }
        return  Math.round(retAngle);
    }

    public void setPosAndAngle(int x, int y, int angle) {
        this.setPos(x, y);
        this.angle = angle;
    }

    public void setPos(Model_iRobot other) {
        this.setPos(other.getPos());
        this.angle = other.angle;
    }

    @Override
    public Point3i predict(double[] noises, double timeSinceUpdate) {
        if(noises.length != 3){
            System.out.println("Incorrect number of noises parameters passed in, please pass in getX noise, getY, noise and angle noise");
            return new Point3i(getX(), getY());
        }
        double xNoise = (getRand()*2*noises[0]) - noises[0];
        double yNoise = (getRand()*2*noises[1]) - noises[1];
        double aNoise = (getRand()*2*noises[2]) - noises[2];

        int dX = 0, dY = 0;
        double dA = 0;
        // Arcing motion
        dA = aNoise + (vRad*timeSinceUpdate);
        dX = (int) (xNoise + Math.cos(Math.toRadians(angle))*(vFwd*timeSinceUpdate));
        dY = (int) (yNoise + Math.sin(Math.toRadians(angle))*(vFwd*timeSinceUpdate));
        x_p = getX() +dX;
        y_p = getY() +dY;
        angle_p = angle+dA;
        return new Point3i(x_p,y_p);
    }

    @Override
    public void collision(Point3i collision_point) {
        // No collision point, set both sensor to false
        if(collision_point == null){
            rightBump = false;
            leftBump = false;
            return;
        }
        if(isFacing(collision_point)){
            if(angleTo(collision_point)%90>(-20)){
                rightBump = true;
            }
            if(angleTo(collision_point)%90<20){
                leftBump = true;
            }
        }
        else{
            rightBump = false;
            leftBump = false;
        }

        //TODO update local map
    }

    @Override
    public void updatePos(boolean followPredict) {
        if(followPredict){
            angle = angle_p;
            setPos(x_p, y_p);
        }
    }

    @Override
    public boolean inMotion() {
        return (vFwd != 0 || vRad != 0);
    }

    @Override
    public void updateSensor(ObstacleList obspoint_positions, PositionList<ItemPosition> sensepoint_positions) {
        /*
        for(ItemPosition other : sensepoint_positions.getList()) {
            if(distanceTo(other) < 600) {
                if(!obspoint_positions.badPath(this, other)) {
                    circleSensor = true;
                    return;
                }
            }
        }
        */
    }

    public enum Type {
    	// marks the unknown obstacle when collide, redo path planning (get around the obstacle) to reach the goal
        GET_TO_GOAL,

		// explore the shape of the unknown obstacle and sent out the shape to others
        EXPLORE_AREA,

		// acts as simple moving obstacle
        RANDOM_MOVING_OBSTACLE,

		// acts as AI opponent try to block robots getting to the goal
        ANTI_GOAL
    }
}
