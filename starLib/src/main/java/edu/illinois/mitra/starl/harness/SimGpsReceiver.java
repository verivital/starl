package edu.illinois.mitra.starl.harness;

import java.util.Vector;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.GpsReceiver;
import edu.illinois.mitra.starl.interfaces.RobotEventListener.Event;
import edu.illinois.mitra.starl.models.Model;
import edu.illinois.mitra.starl.models.Model_GhostAerial;
import edu.illinois.mitra.starl.models.Model_Mavic;
import edu.illinois.mitra.starl.models.Model_iRobot;
import edu.illinois.mitra.starl.models.Model_quadcopter;
import edu.illinois.mitra.starl.models.Model_3DR;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PositionList;

public class SimGpsReceiver implements GpsReceiver {
	private static final String TAG = "GPSReceiver";
	
	private GlobalVarHolder gvh;
	
	public boolean inMotion = false;
	
	private SimGpsProvider provider;
	
	public SimGpsReceiver(GlobalVarHolder gvh, SimGpsProvider provider, Model initpos) {
		this.gvh = gvh;
		this.provider = provider;
		provider.registerReceiver(gvh.id.getName(), this);
		provider.addRobot(initpos);
	}	
	
	@Override
	public void start() {
		// TODO Auto-generated method stub
	}

	@Override
	public void cancel() {
		// TODO Auto-generated method stub
	}

	public void receivePosition(boolean inMotion) {
        if(gvh.trace != null) {
            gvh.trace.traceEvent(TAG, "Received Position", gvh.gps.getMyPosition(), gvh.time());
        }
		gvh.sendRobotEvent(Event.GPS_SELF);
		if(inMotion) {
			gvh.sendRobotEvent(Event.MOTION, Common.MOT_STRAIGHT);
		} else {
			gvh.sendRobotEvent(Event.MOTION, Common.MOT_STOPPED);
		}
		this.inMotion = inMotion;

        if(gvh.gps.getMyPosition() instanceof Model_iRobot){
            Model_iRobot log_p = (Model_iRobot) gvh.gps.getMyPosition();
            gvh.log.i("POSITION", log_p.name + " " + log_p.getX() + " " + log_p.getY() + " " + log_p.getZ() + " " +log_p.angle);
        }
        else if(gvh.gps.getMyPosition() instanceof Model_quadcopter){
			Model_quadcopter log_p = (Model_quadcopter) gvh.gps.getMyPosition();
			gvh.log.i("POSITION", log_p.name + " " + log_p.getX() + " " + log_p.getY() + " " + log_p.getZ() + " " + log_p.getYaw() + " " + log_p.getPitch() + " " + log_p.getRoll() + " " + log_p.getGaz());
		}

		else if(gvh.gps.getMyPosition() instanceof Model_GhostAerial){
			Model_GhostAerial log_p = (Model_GhostAerial) gvh.gps.getMyPosition();
			gvh.log.i("POSITION", log_p.name + " " + log_p.getX() + " " + log_p.getY() + " " + log_p.getZ() + " " + log_p.getYaw() + " " + log_p.getPitch() + " " + log_p.getRoll() + " " + log_p.getGaz());
		}
		else if(gvh.gps.getMyPosition() instanceof Model_Mavic){
			Model_Mavic log_p = (Model_Mavic) gvh.gps.getMyPosition();
			gvh.log.i("POSITION", log_p.name + " " + log_p.getX() + " " + log_p.getY() + " " + log_p.getZ() + " " + log_p.getYaw() + " " + log_p.getPitch() + " " + log_p.getRoll() + " " + log_p.getGaz());
		}
		else if(gvh.gps.getMyPosition() instanceof Model_3DR){
			Model_3DR log_p = (Model_3DR) gvh.gps.getMyPosition();
			gvh.log.i("POSITION", log_p.name + " " + log_p.getX() + " " + log_p.getY() + " " + log_p.getZ() + " " + log_p.getYaw() + " " + log_p.getPitch() + " " + log_p.getRoll() + " " + log_p.getGaz());
		}
	}

	@Override
	public PositionList<ItemPosition> get_robots() {
		return provider.getAllPositions();
	}

	@Override
	public PositionList<ItemPosition> getWaypoints() {
		return provider.getWaypointPositions();
	}
	@Override
	public ObstacleList getObspoints() {
		return provider.getObspointPositions();
	}

	@Override
	public Vector<ObstacleList> getViews() {
		return provider.getViews() ;
	}

	@Override
	public PositionList<ItemPosition> getSensepoints() {
		return provider.getSensePositions();
	}
}
