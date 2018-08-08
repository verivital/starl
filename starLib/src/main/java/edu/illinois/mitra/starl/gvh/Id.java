package edu.illinois.mitra.starl.gvh;

import java.util.Map;
import java.util.Set;

/**
 * Maintains identities of participating robots. Instantiated in the GlobalVarHolder
 * 
 * @author Adam Zimmerman
 * @version 1.0
 * @see GlobalVarHolder
 *
 */
public class Id {
	// Identification
	private Map<String, String> participants = null;
	private String name = null;
	private int idNumber;

	public Id(String name, Map<String, String> participants) {
		this.participants = participants;
		this.name = name;
		String intValue = name.replaceFirst("[^0-9]+", "");
		this.idNumber = Integer.parseInt(intValue);
	}

	public Set<String> getParticipants() {
		return participants.keySet();
	}

	public Map<String,String> getParticipantsIPs() {
		return participants;
	}
	
	public String getName() {
		return name;
	}

	public int getIdNumber() { return idNumber; }
}
