package edu.illinois.mitra.starl.objects;

import android.support.annotation.NonNull;

import java.util.HashMap;

import edu.illinois.mitra.starl.interfaces.Traceable;

/**
 * Represents a 3-dimensional immutable vector with int elements.
 */
public final class Vector3i implements Traceable {

    private final int x;
    private final int y;
    private final int z;

    /**
     * Construct a Vector3i with default value (0, 0, 0).
     */
    public Vector3i() {
        this(0, 0, 0);
    }

    /**
     * Construct a Vector3i with the given getX and getY values, setting getZ to 0.
     */
    public Vector3i(int x, int y) {
        this(x, y, 0);
    }

    /**
     * Construct a Vector3i with the given getX, getY, and getZ values.
     */
    public Vector3i(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Construct a Vector3i with the values of other.
     * @param other Another Vector3i instance.
     */
    public Vector3i(@NonNull Vector3i other) {
        this(other.getX(), other.getY(), other.getZ());
    }

    /**
     * Add vec to this.
     * @return a new Vector3i
     */
    public Vector3i add(@NonNull Vector3i vec) {
        return new Vector3i(x + vec.x, y + vec.y, z + vec.z);
    }

    /**
     * Subtract vec from this.
     * @return a new Vector3i
     */
    public Vector3i subtract(@NonNull Vector3i vec) {
        return new Vector3i(x - vec.x, y - vec.y, z - vec.z);
    }

    /**
     * Scale this by scalar, rounding final values to the nearest int.
     * @return a new Vector3i
     */
    public Vector3i scale(double scalar) {
        return new Vector3i((int)Math.round(x * scalar), (int)Math.round(y * scalar), (int)Math.round(z * scalar));
    }

    /**
     * @return the length of the vector
     */
    public double magnitude() {
        return Math.sqrt(magnitudeSq());
    }

    public double magnitude2D() {
        return Math.sqrt(magnitudeSq2D());
    }

    public int magnitudeSq() {
        return x * x + y * y + z * z;
    }

    public int magnitudeSq2D() {
        return x * x + y * y;
    }

    public Vector3f toVector3f() {
        return new Vector3f(x, y, z);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj != null && obj instanceof Vector3i) {
            Vector3i point = (Vector3i)obj;
            return getX() == point.getX() && getY() == point.getY() && getZ() == point.getZ();
        }
        return false;
    }

    public boolean isZero() {
        return x == 0 && y == 0 && z == 0;
    }

    @Override
    public String toString() {
        return getX() + ", " + getY() + ", " + getZ();
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

    public int getY() {
        return y;
    }

    public int getZ() {
        return z;
    }
}
