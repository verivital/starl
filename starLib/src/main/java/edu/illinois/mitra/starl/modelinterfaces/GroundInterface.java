package edu.illinois.mitra.starl.modelinterfaces;

import edu.illinois.mitra.starl.modelinterfaces.ModelInterface;

public interface GroundInterface extends ModelInterface {

    void sendCurve(int velocity, int radius);

    void sendStraight(int velocity);

    void sendTurn(int velocity, int angle);

    void sendReset();

    void disconnect();

}
