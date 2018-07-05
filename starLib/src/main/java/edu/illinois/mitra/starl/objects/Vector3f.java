package edu.illinois.mitra.starl.objects;

import android.support.annotation.NonNull;

import java.util.HashMap;

import edu.illinois.mitra.starl.interfaces.Traceable;

/**
 * Represents a 3-dimensional immutable vector with double elements.
 */
public final class Vector3f implements Traceable {

    private final double x;
    private final double y;
    private final double z;

    /**
     * Construct a Vector3f with default value (0, 0, 0).
     */
    public Vector3f() {
        this(0, 0, 0);
    }

    /**
     * Construct a Vector3f with the given getX and getY values, setting getZ to 0.
     */
    public Vector3f(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Construct a Vector3f with the given getX, getY, and getZ values.
     */
    public Vector3f(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Construct a Vector3f with the values of other.
     * @param other Another Vector3f instance.
     */
    public Vector3f(@NonNull Vector3f other) {
        this(other.getX(), other.getY(), other.getZ());
    }

    /**
     * Add vec to this.
     * @return a new Vector3f
     */
    public Vector3f add(@NonNull Vector3f vec) {
        return new Vector3f(x + vec.x, y + vec.y, z + vec.z);
    }

    /**
     * Subtract vec from this.
     * @return a new Vector3f
     */
    public Vector3f subtract(@NonNull Vector3f vec) {
        return new Vector3f(x - vec.x, y - vec.y, z - vec.z);
    }

    /**
     * Scale this by scalar.
     * @return a new Vector3f
     */
    public Vector3f scale(double scalar) {
        return new Vector3f(x * scalar, y * scalar, z * scalar);
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

    public double magnitudeSq() {
        return x * x + y * y + z * z;
    }

    public double magnitudeSq2D() {
        return x * x + y * y;
    }

    public Vector3i toVector3i() {
        return new Vector3i((int)Math.round(x), (int)Math.round(y), (int)Math.round(z));
    }

    @Override
    public boolean equals(Object obj) {
        if (obj != null && obj instanceof Vector3f) {
            Vector3f point = (Vector3f)obj;
            return getX() == point.getX() && getY() == point.getY() && getZ() == point.getZ();
        }
        return false;
    }

    public boolean isZero() {
        return magnitudeSq() < 1E-12;
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

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}
