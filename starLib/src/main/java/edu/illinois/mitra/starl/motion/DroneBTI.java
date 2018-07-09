package edu.illinois.mitra.starl.motion;

public interface DroneBTI extends BTI {

    void setControlInput(double yaw, double pitch, double roll, double gaz);

    void sendTakeoff();

    void sendLanding();

    void sendEmergency();

    void hover();

    void setMaxTilt(float maxTilt);

    void disconnect();
}
