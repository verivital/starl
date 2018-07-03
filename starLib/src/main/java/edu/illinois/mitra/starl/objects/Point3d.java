package edu.illinois.mitra.starl.objects;

import java.util.HashMap;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
import edu.illinois.mitra.starl.interfaces.Traceable;

/**
 * This class represents a point in XYZ plane.
 * Robots or any other points with extra properties should be sub classed from this class
 * @author Yixiao Lin
 * @version 1.0
 */

public class Point3d implements Traceable {

	public int x;
	public int y;
	public int z;

	/**
	 * Construct a Point3d with default value (0, 0, 0).
	 */
	public Point3d(){
		set(0, 0, 0);
	}

	/**
	 * Construct a Point3d with the given x and y values, setting z to 0.
	 */
	public Point3d(int x, int y) {
		set(x, y, 0);
	}

	/**
	 * Construct a Point3d with the given x, y, and z values.
	 */
	public Point3d(int x, int y, int z) {
		set(x, y, z);
	}

	/**
	 * Construct a Point3d with the values of other.
	 * @param other Another Point3d instance.
	 */
	public Point3d(Point3d other) {
		set(other.x(), other.y(), other.z());
	}

	/**
	 * Set the Point3d's values to the given values.
	 * @return a reference to this
	 */
	private Point3d set(int x, int y, int z) {
		this.x(x);
		this.y(y);
		this.z(z);
		return this;
	}

	/**
	 * Add vec to this.
	 * @return a reference to this
	 */
	public final Point3d add(Vector3d vec) {
		return set(x + vec.x(), y + vec.y(), z + vec.z());
	}

	/**
	 * Subtract vec from this.
	 * @return a reference to this
	 */
	public final Point3d subtract(Vector3d vec) {
		return set(x - vec.x(), y - vec.y(), z - vec.z());
	}

	/**
	 * @return a new vector from point to this.
	 */
	public final Vector3d subtract(Point3d point) {
		return new Vector3d(x - point.x, y - point.y, z - point.z);
	}
	
	/**
	 * @param other The Point3d to measure against
	 * @return Euclidean distance to Point3d other
	 */
	public double distanceTo(Point3d other) {
		return subtract(other).magnitude();
	}

    public double distanceTo2D(Point3d other) {
        return subtract(other).magnitude2d();
    }
	
	@Override
	public boolean equals(Object obj) {
		if (obj != null && obj instanceof Point3d) {
			Point3d point = (Point3d)obj;
			return x() == point.x() && y() == point.y() && z() == point.z();
		}
		return false;
	}

	@Override
	public String toString() {
		return "Point3d: " + x() + ", " + y() + ", " + z();
	}

	@Override
	public HashMap<String, Object> getXML() {
		HashMap<String, Object> retval = new HashMap<String,Object>();
		retval.put("name", ' ');
		retval.put("x", x());
		retval.put("y", y());
		retval.put("z", z());
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
	
	public final int x(){
		return x;
	}
	
	public final int y(){
		return y;
	}
	public final int z(){
		return z;
	}

	public final void x(int x) {
		this.x = x;
	}

	public final void y(int y) {
		this.y = y;
	}

	public final void z(int z) {
		this.z = z;
	}
}

