package edu.illinois.mitra.starl.objects;

import android.support.annotation.NonNull;

/**
 * This class represents an immutable point in XYZ plane.
 * @author Yixiao Lin
 * @version 1.0
 */

public final class Point3i {

	private final int x;
	private final int y;
	private final int z;

	public final int getX(){
		return x;
	}
	public final int getY(){
		return y;
	}
	public final int getZ(){
		return z;
	}

	/**
	 * Construct a Point3i with default value (0, 0, 0).
	 */
	public Point3i(){
		this(0, 0, 0);
	}

	/**
	 * Construct a Point3i with the given getX and getY values, setting getZ to 0.
	 */
	public Point3i(int x, int y) {
		this(x, y, 0);
	}

	/**
	 * Construct a Point3i with the given getX, getY, and getZ values.
	 */
	public Point3i(int x, int y, int z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Construct a Point3i with the values of other.
	 * @param other Another Point3i instance.
	 */
	public Point3i(@NonNull Point3i other) {
		this(other.getX(), other.getY(), other.getZ());
	}

	/**
	 * Add vec to this.
	 * @return a reference to this
	 */
	public final Point3i add(@NonNull Vector3i vec) {
		return new Point3i(x + vec.getX(), y + vec.getY(), z + vec.getZ());
	}

	/**
	 * Subtract vec from this.
	 * @return a reference to this
	 */
	public final Point3i subtract(@NonNull Vector3i vec) {
		return new Point3i(x - vec.getX(), y - vec.getY(), z - vec.getZ());
	}

	/**
	 * @return a new vector from point to this.
	 */
	public final Vector3i subtract(@NonNull Point3i point) {
		return new Vector3i(x - point.x, y - point.y, z - point.z);
	}
	
	/**
	 * @param other The Point3i to measure against
	 * @return Euclidean distance to Point3i other
	 */
	public double distanceTo(@NonNull Point3i other) {
		return subtract(other).magnitude();
	}

    public double distanceTo2D(@NonNull Point3i other) {
        return subtract(other).magnitude2D();
    }
	
	@Override
	public boolean equals(Object obj) {
		if (obj != null && obj instanceof Point3i) {
			Point3i point = (Point3i)obj;
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

	/*
	@Override
	public HashMap<String, Object> getXML() {
		HashMap<String, Object> retval = new HashMap<String,Object>();
		retval.put("name", ' ');
		retval.put("getX", getX());
		retval.put("getY", getY());
		retval.put("getZ", getZ());
		return retval;
	}
	*/
	
	// Hashing and equals checks are done only against the position's name. Position names are unique!
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + toString().hashCode();
		return result;
	}
}

