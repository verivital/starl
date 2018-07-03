package edu.illinois.mitra.starl.models;

import java.util.Random;

import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3d;
import edu.illinois.mitra.starl.objects.PositionList;

public abstract class Model extends ItemPosition {

	private final Random rand = new Random();

	protected final double rand() {
		return rand.nextDouble();
	}

	public abstract Point3d predict(double[] noises, double timeSinceUpdate);
	public abstract void collision(Point3d collision_point);
	public abstract void updatePos(boolean followPredict);
	public abstract boolean inMotion();
	public abstract void updateSensor(ObstacleList obspoint_positions, PositionList<ItemPosition> sensepoint_positions);

	public Model() {}

	public Model(String name, int x, int y) {
		super(name, x, y);
	}

	public Model(String name, int x, int y, int z) {
		super(name, x, y, z);
	}

	public Model(ItemPosition t_pos) {
		super(t_pos);
	}

    // Used to identify the real type of a Model
	public final String getTypeName() {
		return getClass().getSimpleName();
	}

    // Subclass-specific constants that must be set
    public abstract int radius();
}
