package edu.illinois.mitra.starl.modelinterfaces;

import edu.illinois.mitra.starl.modelinterfaces.ModelInterface;

public interface DroneInterface extends ModelInterface {

    void setControlInput(double yaw, double pitch, double roll, double gaz);

    void sendTakeoff();

    void sendLanding();

    void sendEmergency();

    void hover();

    void setMaxTilt(float maxTilt);

    void disconnect();
}
