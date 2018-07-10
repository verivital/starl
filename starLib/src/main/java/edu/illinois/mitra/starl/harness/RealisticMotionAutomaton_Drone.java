package edu.illinois.mitra.starl.harness;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.RobotEventListener;
import edu.illinois.mitra.starl.motion.BTI;
import edu.illinois.mitra.starl.motion.BTI;
import edu.illinois.mitra.starl.motion.DroneBTI;
import edu.illinois.mitra.starl.motion.MotionAutomaton_Drone;
import edu.illinois.mitra.starl.motion.MotionParameters;


/**
 * Motion Automaton class for drones that extends MotionAutomaton_Drone. Used to send motion commands via bluetooth for real applications.
 * Each robot model has a unique bluetooth interface.
 */
public class RealisticMotionAutomaton_Drone extends MotionAutomaton_Drone {
    private DroneBTI bti;

    public RealisticMotionAutomaton_Drone(GlobalVarHolder gvh, BTI bti) {
        super(gvh);
        this.bti = (DroneBTI)bti;
    }

    @Override
    protected void rotateDrone(){
        bti.setControlInput(rescale(calculateYaw(), 5), 0, 0, 0);
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
        bti.setControlInput(0,0,0,0);
        gvh.log.i(TAG, "Drone hovering");
    }

    protected void setMaxTilt(float val) {
        bti.setMaxTilt(val);
    }

    @Override
    public void setParameters(MotionParameters param) {
        // TODO Auto-generated method stub
    }

    protected double rescale(double value, double max_value){
        if(Math.abs(value) > max_value){
            return (Math.signum(value));
        }
        else{
            return value/max_value;
        }
    }

    public void takePicture(){}
}
