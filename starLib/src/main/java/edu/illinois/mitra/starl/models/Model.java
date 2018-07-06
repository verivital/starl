package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.motion.BTI;
import edu.illinois.mitra.starl.motion.BluetoothInterface;
import edu.illinois.mitra.starl.motion.DjiController;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starl.objects.PositionList;
import java.util.*;

public abstract class Model extends ItemPosition {

	private Random rand = new Random();

	protected double getRand(){
		return rand.nextDouble();
	}

	public abstract Point3i predict(double[] noises, double timeSinceUpdate);
	public abstract void collision(Point3i collision_point);
	public abstract void updatePos(boolean followPredict);
	public abstract boolean inMotion();
	public abstract void updateSensor(ObstacleList obspoint_positions, PositionList<ItemPosition> sensepoint_positions);
	public abstract int radius();

	// to replace if-else chain with instanceof to determine the bluetooth interface for a particular model.
	// This would be a static factory function if it didn't have to be polymorphic.
	public abstract Class<? extends BTI> getBluetoothInterface();

	public Model() {

	}
	public Model(String name, int x, int y){
		super(name, x, y);
	}

	public Model(String name, int x, int y, int z) {
		super(name, x, y, z);
	}

	public Model(ItemPosition t_pos) {
		super(t_pos);
	}

	public final String getTypeName(){
	    return getClass().getSimpleName();
    }
}
