package edu.illinois.mitra.starl.motion;

public class SimDroneBTI implements DroneBTI {

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

    void sendTakeoff();

    void sendLanding();

    void sendEmergency();

    void hover();

    void setMaxTilt(float maxTilt);

    void disconnect();

}
