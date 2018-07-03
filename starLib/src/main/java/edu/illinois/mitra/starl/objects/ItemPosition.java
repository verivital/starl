package edu.illinois.mitra.starl.objects;

import java.util.HashMap;

import edu.illinois.mitra.starl.exceptions.ItemFormattingException;
//import edu.illinois.mitra.starl.interfaces.Traceable;
/**
 * This class represents the position of a point in XYZ plane.
 * Robots or any other points with extra properties should be sub classed from this class
 * @author Yixiao Lin, Adam Zimmerman
 * @version 2.0
 */
public class ItemPosition extends Point3d implements Comparable<ItemPosition>{
//	private static final String TAG = "itemPosition";
//	private static final String ERR = "Critical Error";
	
	public String name;
	public int index;
	public long receivedTime;

	public ItemPosition(){
		super();
		setName("");
	}

	/**
	 * Construct an ItemPosition from a name, X, and Y positions, With Z= 0 as default
	 *
	 * @param name The name of the new position
	 * @param x X position
	 * @param y Y position
	 */
	public ItemPosition(String name, int x, int y) {
		super(x, y);
		setName(name);
	}
	
	public ItemPosition(String name, int x, int y, int z) {
		super(x, y, z);
		setName(name);
	}
	
	public ItemPosition(String name, int x, int y, int z, int index) {
		super(x, y, z);
		setName(name);
		this.index = index;
	}
	
	/**
	 * Construct an ItemPosition by cloning another
	 * Do not use this method to clone robots, it will only clone name, position and heading
	 * @param other The ItemPosition to clone
	 */
	
	public ItemPosition(ItemPosition other) {
		super(other);
		setName(other.name);
	}
	
	/**
	 * Construct an ItemPosition from a received GPS broadcast message
	 * 
	 * @param received GPS broadcast received 
	 * @throws ItemFormattingException
	 */
	public ItemPosition(String received) throws ItemFormattingException {
		String[] parts = received.replace(",", "").split("\\|");
		if(parts.length == 7) {
			this.name = parts[1];
			this.setX(Integer.parseInt(parts[2]));
			this.setY(Integer.parseInt(parts[3]));
			this.setZ(Integer.parseInt(parts[4]));
			this.index = Integer.parseInt(parts[5]);
		} else {
			throw new ItemFormattingException("Should be length 7, is length " + parts.length);
		}
	}
	
	
	@Override 
	public String toString() {
		return name + ": " + getX() + ", " + getY() + ", " + getZ() + ". index " + index;
	}
	
	// Hashing and equals checks are done only against the position's name. Position names are unique!
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((name == null) ? 0 : name.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ItemPosition other = (ItemPosition) obj;
		if(other.getX() != this.getX() || other.getY() != this.getY() || other.getZ() != this.getZ()){
			return false;
		}
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		return true;

		
	}

	@Override
	public HashMap<String, Object> getXML() {
		HashMap<String, Object> retval = new HashMap<String,Object>();
		retval.put("name", name);
		retval.put("getX", getX());
		retval.put("getY", getY());
		retval.put("getZ", getZ());
		return retval;
	}
	
	public String toMessage() {
		return getX() + "," + getY() + "," + getZ() + "," + name +","+index;
	}
	

	public static ItemPosition fromMessage(String msg) {
		String[] parts = msg.split(",");
		if(parts.length != 5)
			throw new IllegalArgumentException("Can not parse ItemPosition from " + msg + ".");
		
		return new ItemPosition(parts[4], Integer.parseInt(parts[0]), Integer.parseInt(parts[1]), Integer.parseInt(parts[2]), Integer.parseInt(parts[3]));
	}

	public int compareTo(ItemPosition other) {
		return name.compareTo(other.name);
	}
	
	private void setName(String name){
		if(name == null){
			this.name = "";
			return;
		}
		this.name = name.split(",", 2)[0];
	}
	
	public String getName(){
		return name;
	}
	
	public int getIndex(){
		return index;
	}
	
}
