package edu.illinois.mitra.starl.motion;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.modelinterfaces.DroneInterface;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Drone;


/**
 * Motion Automaton class for drones that extends MotionAutomaton_Drone. Used to send motion commands via bluetooth for real applications.
 * Each robot model has a unique bluetooth interface.
 */
public class RealMotionAutomaton_Drone extends MotionAutomaton_Drone {
    private DroneInterface bti;

    public RealMotionAutomaton_Drone(GlobalVarHolder gvh, DroneInterface bti) {
        super(gvh);
        this.bti = bti;
    }

    @Override
    protected void setControlInput(double yaw_v, double pitch, double roll, double gaz){
        bti.setControlInput(yaw_v, pitch, roll, gaz);
        gvh.log.i(TAG, "control input as, yaw, pitch, roll, thrust " + yaw_v + ", " + pitch + ", " +roll + ", " +gaz);
    }

    /**
     *  	take off from ground
     */
    protected void takeOff(){
        bti.sendTakeoff();      //Bluetooth command to control the drone
        gvh.log.i("POSITION DEBUG", "Drone taking off");
    }

    /**
     * land on the ground
     */
    protected void land(){
        bti.sendLanding();
        gvh.log.i(TAG, "Drone landing");
    }

    /**
     * hover at current position
     */
    protected void hover(){
        //Bluetooth command to control the drone
        storeControlInput(0,0,0,0);
        gvh.log.i(TAG, "Drone hovering");
    }

    protected void setMaxTilt(float val) {
        bti.setMaxTilt(val);
    }
}
