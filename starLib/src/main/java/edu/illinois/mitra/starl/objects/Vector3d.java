package edu.illinois.mitra.starl.objects;

import java.util.HashMap;

import edu.illinois.mitra.starl.interfaces.Traceable;

public final class Vector3d implements Traceable {

    private int x;
    private int y;
    private int z;

    /**
     * Construct a Vector3d with default value (0, 0, 0).
     */
    public Vector3d() {
        set(0, 0, 0);
    }

    /**
     * Construct a Vector3d with the given getX and getY values, setting getZ to 0.
     */
    public Vector3d(int x, int y) {
        set(x, y, 0);
    }

    /**
     * Construct a Vector3d with the given getX, getY, and getZ values.
     */
    public Vector3d(int x, int y, int z) {
        set(x, y, z);
    }

    /**
     * Construct a Vector3d with the values of other.
     * @param other Another Vector3d instance.
     */
    public Vector3d(Vector3d other) {
        set(other.getX(), other.getY(), other.getZ());
    }

    /**
     * Set the Vector3d's values to the given values.
     * @return a reference to this
     */
    public Vector3d set(int x, int y, int z) {
        this.setX(x);
        this.setY(y);
        this.setZ(z);
        return this;
    }

    /**
     * Add vec to this.
     * @return a reference to this
     */
    public Vector3d add(Vector3d vec) {
        return set(x + vec.x, y + vec.y, z + vec.z);
    }

    /**
     * Subtract vec from this.
     * @return a reference to this
     */
    public Vector3d subtract(Vector3d vec) {
        return set(x - vec.x, y - vec.y, z - vec.z);
    }

    /**
     * Scale this by scalar, rounding final values to the nearest int.
     * @return a reference to this
     */
    public Vector3d scale(double scalar) {
        return set((int)(x * scalar), (int)(y * scalar), (int)(z * scalar));
    }

    /**
     * @return the length of the vector
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double magnitude2d() {
        return Math.sqrt(x * x + y * y);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj != null && obj instanceof Vector3d) {
            Vector3d point = (Vector3d)obj;
            return getX() == point.getX() && getY() == point.getY() && getZ() == point.getZ();
        }
        return false;
    }

    @Override
    public String toString() {
        return "Vector3d: " + getX() + ", " + getY() + ", " + getZ();
    }

    @Override
    public HashMap<String, Object> getXML() {
        HashMap<String, Object> retval = new HashMap<String,Object>();
        retval.put("name", ' ');
        retval.put("getX", getX());
        retval.put("getY", getY());
        retval.put("getZ", getZ());
        return retval;
    }

    // Hashing and equals checks are done only against the position's name. Position names are unique!
    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + toString().hashCode();
        return result;
    }

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public int getZ() {
        return z;
    }

    public void setZ(int z) {
        this.z = z;
    }
}
