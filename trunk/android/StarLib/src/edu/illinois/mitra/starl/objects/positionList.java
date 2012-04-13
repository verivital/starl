
package edu.illinois.mitra.starl.objects;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * PositionList is a thin wrapper for a HashMap (String -> ItemPosition). Collections of ItemPositions
 * are stored in PositionLists.
 * @author Adam Zimmerman
 * @version 1.0
 */
public class PositionList {
	private static final String TAG = "positionList";
	private static final String ERR = "Critical Error";
	
	private HashMap<String,ItemPosition> positions;
	
	/**
	 * Create an empty PositionList
	 */
	public PositionList() {
		positions = new HashMap<String,ItemPosition>();
	}
	
	/**
	 * @param received The ItemPosition to add to the list. If a position with the same name is present, it
	 * will be overwritten. 
	 */
	public void update(ItemPosition received) {
		positions.put(received.name, received);
	}
	
	/**
	 * @param name The name to match
	 * @return An ItemPosition with a matching name, null if one doesn't exist.
	 */
	public ItemPosition getPosition(String name) {
		if(positions.containsKey(name)) {
			return positions.get(name);
		}
		return null;
	}
	
	/**
	 * @param exp The regex string to match against
	 * @return The first ItemPosition in the PositionList whose name matches the regular expression
	 */
	public ItemPosition getPositionRegex(String exp) {
		for(String n : positions.keySet()) {
			if(n.matches(exp)) {
				return positions.get(n);
			}
		}
		return null;
	}
	
	public boolean hasPositionFor(String name) {
		return positions.containsKey(name);
	}

	@Override
	public String toString() {
		String toRet = "";
		for(ItemPosition i : positions.values()) {
			toRet = toRet + i.toString() + "\n";
		}
		return toRet;
	}
	
	public int getNumPositions() {
		return positions.size();
	}
	
	/**
	 * @return An ArrayList representation of all contained ItemPositions
	 */
	public ArrayList<ItemPosition> getList() {
		return new ArrayList<ItemPosition>(positions.values());
	}
}