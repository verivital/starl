package edu.illinois.mitra.starl.harness;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.models.Model_Drone;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Drone;


/**
 * Motion Automaton class for drones that extends MotionAutomaton_Drone.
 * Used to send motion commands to the SimGpsProvider to simulate motion.
 */
public class RealisticSimMotionAutomaton_Drone extends MotionAutomaton_Drone {
    private SimGpsProvider gpsp;
    private String name;
    private String typeName;

    public RealisticSimMotionAutomaton_Drone(GlobalVarHolder gvh, SimGpsProvider gpsp) {
        super(gvh);
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
    protected void rotateDrone(){
        setControlInput(rescale(calculateYaw(), 5), 0, 0, 0);
    }

    @Override
    protected void setMaxTilt(float val){
    //TODO: Implement
    }

}
