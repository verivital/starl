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

    /**
     * @return the length of the vector in two dimensions
     */
    public double magnitude2D() {
        return Math.sqrt(magnitudeSq2D());
    }

    /**
     * @return the squared length of the vector
     */
    public double magnitudeSq() {
        return x * x + y * y + z * z;
    }

    /**
     * @return the squared length of the vector in two dimensions
     */
    public double magnitudeSq2D() {
        return x * x + y * y;
    }

    /**
     * @return a new {@link Vector3i} representing this, with rounded components
     */
    public Vector3i toVector3i() {
        return new Vector3i((int)Math.round(x), (int)Math.round(y), (int)Math.round(z));
    }

    /**
     * @param obj the object to compare equal
     * @return true if obj is a Vector3f and the difference between this and obj
     * is approximately the zero vector.
     * @see #isZero()
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector3f) {
            Vector3f vec = (Vector3f)obj;
            return this.subtract(vec).isZero();
        }
        return false;
    }

    /**
     * @return true if this is approximately equal to the zero vector
     */
    public boolean isZero() {
        return magnitudeSq() < 1E-12;
    }

    /**
     * @return a String with the components, comma-separated
     */
    @Override
    public String toString() {
        return getX() + ", " + getY() + ", " + getZ();
    }

    /**
     * @return a HashMap representing this object
     */
    @Override
    public HashMap<String, Object> getXML() {
        HashMap<String, Object> retval = new HashMap<String,Object>();
        retval.put("name", ' ');
        retval.put("getX", getX());
        retval.put("getY", getY());
        retval.put("getZ", getZ());
        return retval;
    }

    @Override
    public int hashCode() {
        return toString().hashCode();
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
