package edu.illinois.mitra.starl.models;

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

	public Model(){

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
