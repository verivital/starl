package edu.illinois.mitra.starl.models;

/**
 * Created by yangy14 on 6/2/2017.
 * TODO: Cleanup, already modifies Model_Drone.
 */

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.motion.DroneBTI;
import edu.illinois.mitra.starl.motion.o3DRController;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.PIDParams;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starl.objects.PositionList;
import edu.illinois.mitra.starl.objects.Vector3f;
import edu.illinois.mitra.starl.objects.Vector3i;

public class Model_3DR extends Model_Drone {

    static {
        ModelRegistry.register(Model_3DR.class);
    }

    public Model_3DR(String received) throws ItemFormattingException{
        super(received);
    }

    public Model_3DR(String name, int x, int y) {
        super(name, x, y);
    }
/*
    public Model_3DR(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    public Model_3DR(String name, int x, int y, int z, int yaw) {
        super(name, x, y, z, yaw);
    }

    public Model_3DR(String name, int x, int y, int z, double yaw, double pitch, double roll) {
        super(name, x, y, z, yaw, pitch, roll);
    }
    */
    public Model_3DR(ItemPosition t_pos) {
        super(t_pos);
    }

    @Override
    public int radius() { return 340; }

    @Override
    public double height() { return 50; }

    @Override
    public double mass() { return .5; }

    @Override
    public double max_gaz() { return 1000; }

    @Override
    public double max_pitch_roll(){ return 20; }

    @Override
    public double max_yaw_speed() { return 200; }

    @Override
    public Class<? extends DroneBTI> getBluetoothInterface() {
        return o3DRController.class;
    }

    @Override
    public PIDParams getPIDParams() {
        PIDParams p = new PIDParams();
        p.Kp = 0.0714669809792096;
        p.Ki = 0.0110786899216426;
        p.Kd = 0.113205037832174;
        p.saturationLimit = 50;
        p.windUpLimit = 185;
        p.filterLength = 8;
        p.reversed = true;
        return p;
    }
}
