package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.illinois.mitra.starl.objects.PositionList;

/**
 * Created by wangcs2 on 6/2/2017.
 */

public class Model_GhostAerial extends Model_Drone {

    public Model_GhostAerial(String received) throws ItemFormattingException {
        super(received);
    }

    public Model_GhostAerial(String name, int x, int y) {
        super(name, x, y, 0);
    }

    public Model_GhostAerial(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    public Model_GhostAerial(String name, int x, int y, int z, int yaw) {
        super(name, x, y, z, yaw);
    }

    public Model_GhostAerial(String name, int x, int y, int z, double yaw, double pitch, double roll) {
        super(name, x, y, z, yaw, pitch, roll);
    }

    public Model_GhostAerial(ItemPosition t_pos) {
        super(t_pos.name, t_pos.getX(), t_pos.getY(), t_pos.getZ());
    }

    @Override
    public int radius() { return 340; }

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
}
