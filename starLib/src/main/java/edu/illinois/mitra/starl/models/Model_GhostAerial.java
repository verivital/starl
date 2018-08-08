package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.modelinterfaces.DroneInterface;
import edu.illinois.mitra.starl.modelinterfaces.GhostAerialInterface;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.PIDParams;

/**
 * A model of the Ehang GhostDrone 2.0.
 * @see Model_Drone
 */
public class Model_GhostAerial extends Model_Drone {

    @SuppressWarnings("unused")
    public Model_GhostAerial() {}

    @SuppressWarnings("unused")
    public Model_GhostAerial(String received) throws ItemFormattingException {
        super(received);
    }

    @SuppressWarnings("unused")
    public Model_GhostAerial(String name, int x, int y) {
        super(name, x, y);
    }

    @SuppressWarnings("unused")
    public Model_GhostAerial(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    @SuppressWarnings("unused")
    public Model_GhostAerial(ItemPosition t_pos) {
        super(t_pos);
    }

    @Override
    public int radius() { return 340; }

    @Override
    public String ip() { return "10.255.24.0."; }

    @Override
    public double height() { return 50; }

    @Override
    public double mass() { return 1.2; }

    @Override
    public double max_gaz() { return 2000; }

    @Override
    public double max_pitch_roll() { return 20; }

    @Override
    public double max_yaw_speed() { return 200; }

    @Override
    public Class<? extends DroneInterface> getModelInterface() {
        return GhostAerialInterface.class;
    }

    @Override
    public PIDParams getPIDParams() {
        PIDParams p = new PIDParams();
        p.Kp = 2.5E-4;//0.0314669809792096;
        p.Ki = 0.75*p.Kp;//0.0110786899216426;
        p.Kd = 1.5E-3;//0.113205037832174;
        p.saturationLimit = 50;
        p.windUpLimit = 185;
        p.filterLength = 8;
        p.reversed = true;
        return p;
    }
}
