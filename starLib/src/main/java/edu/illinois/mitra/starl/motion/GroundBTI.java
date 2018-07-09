package edu.illinois.mitra.starl.motion;

public interface GroundBTI extends BTI {

    void sendCurve(int velocity, int radius);

    void sendStraight(int velocity);

    void sendTurn(int velocity, int angle);

    void sendReset();

    void disconnect();

}
