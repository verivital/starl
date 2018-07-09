package edu.illinois.mitra.starl.motion;

import android.provider.Settings;

import java.util.Arrays;

import edu.illinois.mitra.starl.interfaces.RobotEventListener;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.motion.GroundBTI;
import edu.illinois.mitra.starl.motion.BluetoothInterface;
import edu.illinois.mitra.starl.motion.BluetoothCommands;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;

public class IRobotBTI implements GroundBTI {
    private BluetoothInterface bti;

    public IRobotBTI(GlobalVarHolder gvh, String mac) {
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
        byte[] reset = new byte[]{(byte) 7};
        bti.send(reset);
    }

    public void disconnect() {
        bti.disconnect();
    }
}
