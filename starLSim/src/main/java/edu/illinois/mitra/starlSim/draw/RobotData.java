package edu.illinois.mitra.starlSim.draw;

import java.awt.Color;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.models.Model_Ground;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;

public class RobotData
{
	private ItemPosition item;
	private Color color;
	private ObstacleList world;

	public RobotData(ItemPosition item) {
		this(item, null);
	}

	public RobotData(ItemPosition item, Color color) {
		this(item, color, null);
	}

	public RobotData(ItemPosition item, Color color, ObstacleList world) {
		if (item == null) {
			throw new IllegalArgumentException("Null item parameter not supported.");
		}
		this.item = item;
		this.color = color != null ? color : Color.black;
		this.world = world != null ? world : new ObstacleList();
	}

	public String getName() {
		return item.getName();
	}

	public boolean isModel() {
		return item instanceof Model;
	}

	public String getTypename() {
		if (isModel()) {
			return ((Model) item).getTypeName();
		}
		throw new IllegalArgumentException("Cannot get typename for a non-model item.");
	}

	public int getX() {
		return item.getX();
	}

	public int getY() {
		return item.getY();
	}

	public int getZ() {
		return item.getZ();
	}

	public boolean isGround() {
		return item instanceof Model_Ground;
	}

	public double getDegrees() {
		if (isGround()) {
			return ((Model_Ground) item).angle;
		}
		throw new IllegalArgumentException("Cannot get degrees for a non-ground item.");
	}

	public boolean isDrone() {
		return item instanceof Model_Drone;
	}

	public double getYaw() {
		if (isDrone()) {
			return ((Model_Drone) item).getYaw();
		}
		throw new IllegalArgumentException("Cannot get yaw for a non-drone item.");
	}

	public double getPitch() {
		if (isDrone()) {
			return ((Model_Drone) item).getPitch();
		}
		throw new IllegalArgumentException("Cannot get pitch for a non-drone item.");
	}

	public double getRoll() {
		if (isDrone()) {
			return ((Model_Drone) item).getRoll();
		}
		throw new IllegalArgumentException("Cannot get roll for a non-drone item.");
	}

	public long getTime() {
		return item.receivedTime;
	}

	public int getRadius() {
		if (isModel()) {
			return ((Model) item).radius();
		}
		throw new IllegalArgumentException("Cannot get radius for a non-model item.");
	}

	public Color getColor() {
		return color;
	}

	public ObstacleList getWorld() {
		return world;
	}

	public Model_Ground.Type getGroundType() {
		if (isGround()) {
			return ((Model_Ground) item).type;
		}
		throw new IllegalArgumentException("Cannot get type for a non-ground item.");
	}

	public boolean getLeftbump() {
		if (isGround()) {
			return ((Model_Ground) item).leftBump;
		}
		throw new IllegalArgumentException("Cannot get leftBump for a non-ground item.");
	}

	public boolean getRightbump() {
		if (isGround()) {
			return ((Model_Ground) item).rightBump;
		}
		throw new IllegalArgumentException("Cannot get rightBump for a non-ground item.");
	}
}
