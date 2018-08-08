
package edu.illinois.mitra.starl.objects;

import java.util.*;
import java.util.Map.Entry;

/**
 * PositionList is a thin wrapper for a HashMap (String -&gt; ItemPosition). Collections of ItemPositions
 * are stored in PositionLists.
 * they are sorted in the natural order of their keys
 * @author Adam Zimmerman, Yixiao Lin
 * @version 1.1
 */
public class PositionList<T extends ItemPosition> implements Iterable<T> {
//	private static final String TAG = "positionList";
//	private static final String ERR = "Critical Error";
	
	private TreeMap<String,T> positions;
	
	/**
	 * Create an empty PositionList
	 */
	public PositionList() {
		positions = new TreeMap<String,T>();
	}
	
	public void update(T received, long time) {
		received.receivedTime = time;
		positions.put(received.name, received);
	}
	
	public void update(T received) {
		update(received, 0);
	}
	
	/**
	 * @param name The name to match
	 * @return An ItemPosition with a matching name, null if one doesn't exist.
	 */
	public T getPosition(String name) {
		if(positions.containsKey(name)) {
			return positions.get(name);
		}
		return null;
	}
	
	/**
	 * @param exp The regex string to match against
	 * @return The first ItemPosition in the PositionList whose name matches the regular expression
	 */
	public T getPositionRegex(String exp) {
		for(Entry<String, T> entry : positions.entrySet()) {
			if(entry.getKey().matches(exp)) {
				return entry.getValue();
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
		for(T i : positions.values()) {
			toRet = toRet + i.toString() + "\n";
		}
		return toRet;
	}
	
	public int getNumPositions() {
		return positions.size();
	}
	
	public void clear() {
		positions.clear();
	}
	
	/**
	 * @return An ArrayList representation of all contained ItemPositions
	 */
	public ArrayList<T> getList() {
		return new ArrayList<T>(positions.values());
	}

	@Override
	public Iterator<T> iterator() {
		return positions.values().iterator();
	}

}