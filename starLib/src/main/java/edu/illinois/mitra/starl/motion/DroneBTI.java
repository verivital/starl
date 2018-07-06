package edu.illinois.mitra.starl.motion;

public interface DroneBTI extends BTI {

    void setControlInput(float yaw, float pitch, float roll, float gaz);
    default void setControlInput(double yaw, double pitch, double roll, double gaz) {
        setControlInput((float)yaw, (float)pitch, (float)roll, (float)gaz);
    }

    void sendTakeoff();

    void sendLanding();

    void sendEmergency();

    void hover();

    void setMaxTilt(float maxTilt);

    void disconnect();
}
