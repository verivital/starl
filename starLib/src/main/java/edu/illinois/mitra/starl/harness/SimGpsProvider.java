package edu.illinois.mitra.starl.harness;

import java.util.Observer;
import java.util.Vector;

import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_3DR;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starl.models.Model_Mavic;
import edu.illinois.mitra.starl.models.Model_Phantom;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.objects.*;

public interface SimGpsProvider {

	public abstract void registerReceiver(String name,
			SimGpsReceiver simGpsReceiver);

	public abstract void addRobot(Model bot);

	// Implemented only by ideal gps provider
	public abstract void setDestination(String name, ItemPosition dest, int vel);
	
	public abstract void setControlInput(String typeName, String name, double v_yaw, double pitch, double roll, double gaz);

	public abstract void halt(String name);

	// Implemented only be realistic gps provider
	public abstract void setVelocity(String typeName, String name, int fwd, int rad);

	public abstract PositionList<ItemPosition> getAllPositions();

	public abstract void setWaypoints(PositionList<ItemPosition> loadedWaypoints);
	
	public abstract void setSensepoints(PositionList<ItemPosition> loadedSensepoints);
	
	public abstract void setObspoints(ObstacleList loadedObspoints);

	public abstract void setViews(ObstacleList environment, int nBots);

	public abstract PositionList<ItemPosition> getWaypointPositions();
	
	public abstract PositionList<ItemPosition> getSensePositions();

	public abstract ObstacleList getObspointPositions();
	
	public abstract Vector<ObstacleList> getViews();
	
	public abstract void start();
	
	public abstract void addObserver(Observer o);

}