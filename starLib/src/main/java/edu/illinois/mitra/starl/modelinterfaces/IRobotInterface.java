package edu.illinois.mitra.starl.modelinterfaces;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;

public class IRobotInterface implements GroundInterface {
    private BluetoothInterface bti;

    public IRobotInterface(GlobalVarHolder gvh, String mac) {
        bti = new BluetoothInterface(gvh, mac);
    }

    public void sendCurve(int velocity, int radius) {
        bti.send(BluetoothCommands.curve(velocity, radius));
    }

    public void sendStraight(int velocity) {
        bti.send(BluetoothCommands.straight(velocity));
    }

    public void sendTurn(int velocity, int angle) {
        bti.send(BluetoothCommands.turn(velocity, angle));
    }

    public void sendReset() {
        // 7 is the reset opcode for the create
        byte[] reset = {7};
        bti.send(reset);
    }

    public void disconnect() {
        bti.disconnect();
    }
}
