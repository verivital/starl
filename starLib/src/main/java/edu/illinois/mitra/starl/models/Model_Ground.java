package edu.illinois.mitra.starl.models;

import edu.illinois.mitra.starl.objects.ItemPosition;

public abstract class Model_Ground extends Model {
    public Model_Ground() {}

    public Model_Ground(String name, int x, int y) {
        super(name, x, y);
    }

    public Model_Ground(String name, int x, int y, int z) {
        super(name, x, y, z);
    }

    public Model_Ground(ItemPosition t_pos) {
        super(t_pos);
    }
}
