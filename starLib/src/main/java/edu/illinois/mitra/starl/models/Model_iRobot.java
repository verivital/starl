package edu.illinois.mitra.starl.models;

import java.util.Random;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * This class represents a simple model of the iRobot Create, including angle, radius, type, velocity, leftbump, rightbump, circleSensor, vFwd, vRad
 * and some prediction on getX and getY based on vFwd and vRad
 *
 * default type:
 *	0: get to goal robot
 *	behavior: marks the unknown obstacle when collide, redo path planning (get around the obstacle)to reach the goal
 *	1: explore the area robot
 *	behavior: explore the shape of the unknown obstacle and sent out the shape to others
 *	2: random moving obstacle robot
 *	behavior:acts as simple moving obstacle
 *	3: anti goal robot
 *	behavior:acts as AI opponent try to block robots getting to the goal
 * @author Yixiao Lin
 * @version 1.0
 */
public class Model_iRobot extends Model_Ground {

    public enum Type {
        GET_TO_GOAL,
        EXPLORE_AREA,
        RANDOM_MOVING_OBSTACLE,
        ANTI_GOAL
    }

	public double angle;
	public Type type;
	public double velocity;

	public boolean leftbump;
	public boolean rightbump;
	public boolean circleSensor;

	public double vFwd;
	public double vRad;
	public Random rand;
	public int x_p;
	public int y_p;
	public double angle_p;

	/**
	 * Construct an Model_iRobot from a received GPS broadcast message
	 *
	 * @param received GPS broadcast received
	 * @throws ItemFormattingException
	 */

	public Model_iRobot(String received) throws ItemFormattingException{
		initial_helper();
		String[] parts = received.replace(",", "").split("\\|");
		if(parts.length == 7) {
			this.name = parts[1];
			this.setPos(Integer.parseInt(parts[2]),
					Integer.parseInt(parts[3]),
					Integer.parseInt(parts[4]));
			this.angle = Integer.parseInt(parts[5]);
//		} else {
//			throw new ItemFormattingException("Should be length 7, is length " + parts.length);
//        String[] parts = received.replace(",", "").split("\\|");
//        if(parts.length == 6) {
//            this.angle = Double.parseDouble(parts[4]);
		}
        else {
            throw new ItemFormattingException("Should be length 5, is length " + parts.length);
        }
	}

	public Model_iRobot(String name, int x, int y) {
		super(name, x, y);
		initial_helper();
	}

	public Model_iRobot(String name, int x, int y, double angle) {
		super(name, x, y);
		initial_helper();
		this.angle = angle;
	}

	public Model_iRobot(ItemPosition t_pos) {
		super(t_pos.name, t_pos.getX(), t_pos.getY(), t_pos.getZ());
		initial_helper();
		this.angle = t_pos.index;
		// TODO Auto-generated constructor stub
	}

	@Override
	public String toString() {
		return name + ": " + getX() + ", " + getY() + ", " + getZ() + ", angle " + angle;
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
		if(other.getX() == this.getX() && other.getY() == this.getY()){
			return true;
		}
    	double angleT = Math.toDegrees(Math.atan2((other.getY() - this.getY()) , (other.getX() - this.getX())));
    	if(angleT  == 90){
    		if(this.getY() < other.getY())
    			angleT = angleT + 90;
    		double temp = this.angle % 360;
    		if(temp > 0)
    			return true;
    		else
    			return false;
    	}
		if(angleT < 0)
		{
			angleT += 360;
		}
		double angleT1, angleT2, angleself;
		angleT1 = (angleT - 90) % 360;
		if(angleT1 < 0)
		{
			angleT1 += 360;
		}
		angleT2 = (angleT + 90) % 360;
		if(angleT2 < 0)
		{
			angleT2 += 360;
		}
		angleself = this.angle % 360;
		if(angleself < 0)
		{
			angleself += 360;
		}
		if(angleT2 <= 180)
		{
			if((angleself < angleT1) && (angleself > angleT2))
				return false;
			else
				return true;
		}
		else
		{
			if(angleself > angleT2 || angleself < angleT1)
				return false;
			else
				return true;

		}
	}

	/**
	 * @param other The ItemPosition to measure against
	 * @return Number of degrees this position must rotate to face position other
	 */
	public int angleTo(ItemPosition other) {
		return angleTo(other.getPos());
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
		retAngle = retAngle%360;
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

	private void initial_helper(){
		angle = 0;
		type = Type.GET_TO_GOAL;
		velocity = 0;
		leftbump = false;
		rightbump = false;
		circleSensor = false;
		vFwd = 0;
		vRad = 0;
	}

	@Override
	public int radius() {
	    return 165;
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
			rightbump = false;
			leftbump = false;
			return;
		}
		if(isFacing(collision_point)){
			if(angleTo(collision_point)%90>(-20)){
				rightbump = true;
			}
			if(angleTo(collision_point)%90<20){
				leftbump = true;
			}
		}
		else{
			rightbump = false;
			leftbump = false;
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

		for(ItemPosition other : sensepoint_positions.getList()) {
			if(distanceTo(other)<600){
				if(!obspoint_positions.badPath(this, other)){
					circleSensor = true;
					return;
				}
			}
		}
		return;
	}
}
