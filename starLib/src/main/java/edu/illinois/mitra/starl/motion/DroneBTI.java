package edu.illinois.mitra.starl.motion;

public interface DroneBTI extends BTI {
    void setPitch(double pitch);

    void setRoll(double roll);

    void setYaw(double yaw);

    void setThrottle(double throttle);

    void sendTakeoff();

    void sendLanding();

    void sendEmergency();

    void hover();

    void setMaxTilt(float maxTilt);

    void disconnect();
}
