package edu.illinois.mitra.starl.harness;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Drone;

//TODO: setControlInput() is not compatible with every drone, see below.

/**
 * RealisticSimMotion
 */
public class RealisticSimMotionAutomaton_Drone extends MotionAutomaton_Drone {
    private SimGpsProvider gpsp;
    private String name;
    private String typeName;

    public RealisticSimMotionAutomaton_Drone(GlobalVarHolder gvh, SimGpsProvider gpsp) {
        super(gvh, null);
        this.name = gvh.id.getName();
        this.gpsp = gpsp;
        this.typeName = drone.getTypeName();
    }

    @Override
    public void setControlInput(double yaw_v, double pitch, double roll, double gaz){
        if(yaw_v > 1 || yaw_v < -1){
            throw new IllegalArgumentException("yaw speed must be between -1 to 1");
        }
        if(pitch > 1 || pitch < -1){
            throw new IllegalArgumentException("pitch must be between -1 to 1");
        }
        if(roll > 1 || roll < -1){
            throw new IllegalArgumentException("roll speed must be between -1 to 1");
        }
        if(gaz > 1 || gaz < -1){
            throw new IllegalArgumentException("gaz, vertical speed must be between -1 to 1");
        }

        //TODO: Have to change SimGpsProvider class, because setControlInput is only for quadCopters, setControlInput3DR is only for 3DR, etc.
        //TODO: In order to change it, need to have one list of all robots just for motion settings, investigate more.
        gpsp.setControlInput(typeName,name, yaw_v*drone.max_yaw_speed(),
                pitch*drone.max_pitch_roll(), roll*drone.max_pitch_roll(), gaz*drone.max_gaz());
    }

    /**
     *  	take off from ground
     */
    @Override
    protected void takeOff(){
        gvh.log.i(TAG, "Drone taking off");
        setControlInput(0, 0, 0, 1);
    }

    /**
     * land on the ground
     */
    @Override
    protected void land(){
        gvh.log.i(TAG, "Drone landing");
        //setControlInput(my_model.yaw, 0, 0, 5);
    }

    /**
     * hover at current position
     */
    @Override
    protected void hover(){
        gvh.log.i(TAG, "Drone hovering");
        setControlInput(0, 0, 0, 0);
    }

    @Override
    public void cancel() {
        super.running = false;
    }
}
