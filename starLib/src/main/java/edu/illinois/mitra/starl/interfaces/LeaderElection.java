package edu.illinois.mitra.starl.interfaces;


public interface LeaderElection extends Cancellable {
	
	/**
	 * @return the name of the elected leader or null if election isn't complete 
	 */
	public abstract String getLeader();

	public abstract int getLeaderID();
	
	/**
	 * Begin the election process
	 */
	public abstract void elect();
}
