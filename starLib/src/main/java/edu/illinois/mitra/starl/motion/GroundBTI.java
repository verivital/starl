package edu.illinois.mitra.starl.motion;

public interface GroundBTI extends BTI {

    void connect();

    void send(byte[] to_send);

    byte[] readBuffer(int n_bytes);

    void disconnect();
}
