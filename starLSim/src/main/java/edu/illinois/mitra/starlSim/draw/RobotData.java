package edu.illinois.mitra.starlSim.draw;

import java.awt.Color;

import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.models.Model_iRobot; // for Model_iRobot.Type

public class RobotData
{
	// TODO: make this class more general for any models
	public String name;
	public int x;
	public int y;
	public int z = 0;
	public double degrees;
	public double yaw;
	public double pitch;
	public double roll;
	public long time;

	// optional
	public int radius;
	public Color c;
	public ObstacleList world;
	public Model_iRobot.Type type = Model_iRobot.Type.GET_TO_GOAL;
	public boolean leftbump;
	public boolean rightbump;
	
	public RobotData(String name, int x, int y, double degrees)
	{
		this.name = name;
		this.x = x;
		this.y = y;
		this.degrees = degrees;
	}
	
	public RobotData(String name, int x, int y, double degrees, Color color) {
		this(name, x, y, degrees);
		this.c = color;
	}
	
	public RobotData(String name, int x, int y, double degrees, Color color, ObstacleList world) {
		this(name, x, y, degrees, color);
		this.world = world;
		
	}
	
	public RobotData(String name, int x, int y, double degrees, Color color, ObstacleList world, boolean leftbump, boolean rightbump) {
		this(name, x, y, degrees, color);
		this.world = world;
		this.leftbump = leftbump;
		this.rightbump = rightbump;
	}
	
	public RobotData(String name, int x, int y, double degrees, long t) {
		this(name, x, y, degrees);
		this.time = t;
	}
	
	public RobotData(String name, int x, int y, int z, double yaw, double pitch, double roll, long t) {
		this.name = name;
		this.x = x;
		this.y = y;
		this.z = z;
		this.yaw = yaw;
		this.pitch = pitch;
		this.roll = roll;
		this.time = t;
	}
	
	public RobotData(String name, int x, int y, int z, double yaw, double pitch, double roll) {
		this.name = name;
		this.x = x;
		this.y = y;
		this.z = z;
		this.yaw = yaw;
		this.pitch = pitch;
		this.roll = roll;
	}
	
	public RobotData(String name, int x, int y, double degrees, long t, Color color) {
		this(name, x, y, degrees, t);
		this.c = color;
	}
	
	public RobotData(String name, int x, int y, double degrees, long t, Color color, ObstacleList world) {
		this(name, x, y, degrees, t, color);
		this.world = world;
	}
	
	public RobotData(String name, int x, int y, double degrees, long t, Color color, ObstacleList world, boolean leftbump, boolean rightbump) {
		this(name, x, y, degrees, t, color);
		this.world = world;
		this.leftbump = leftbump;
		this.rightbump = rightbump;
	}
}
