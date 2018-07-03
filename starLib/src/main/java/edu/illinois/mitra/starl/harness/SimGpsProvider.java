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

	void registerReceiver(String name,
						  SimGpsReceiver simGpsReceiver);

	void addRobot(Model bot);

	// Implemented only by ideal gps provider
	void setDestination(String name, ItemPosition dest, int vel);
	
	void setControlInput(String typename, String name, double v_yaw, double pitch, double roll, double gaz);

	// Implemented only be realistic gps provider
	void setVelocity(String name, int fwd, int rad);

	void halt(String name);
	
	PositionList<ItemPosition> getAllPositions();

	void setWaypoints(PositionList<ItemPosition> loadedWaypoints);
	
	void setSensepoints(PositionList<ItemPosition> loadedSensepoints);
	
	void setObspoints(ObstacleList loadedObspoints);

	void setViews(ObstacleList environment, int nBots);

	PositionList<ItemPosition> getWaypointPositions();
	
	PositionList<ItemPosition> getSensePositions();

	ObstacleList getObspointPositions();
	
	Vector<ObstacleList> getViews();
	
	void start();
	
	void addObserver(Observer o);

}