package edu.illinois.mitra.starl.interfaces;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;

import edu.illinois.mitra.starl.gvh.GlobalVarHolder;

/**
 * StarLCallable is a base class for all Callable classes intended for use in StarL. In a simulation environment, StarLCallable handles registering the created thread with the
 * simulator engine upon calling call(), and removes the registration when the call completes. All StarLCallables return a list of Objects, making it easy to return a large number
 * of different objects. The protected Object[] results variable is provided, along with a method returnResults(), to easily manage the objects to be returned.
 * 
 * @author Adam Zimmerman
 * @version 1.0
 */
public abstract class StarLCallable implements Callable<List<Object>> {
	
	protected GlobalVarHolder gvh;
	protected Object[] results;
	private String threadname;
	protected String name;
	
	/**
	 * Create a new StarLCallable
	 * @param gvh the main global variable holder
	 */
	public StarLCallable(GlobalVarHolder gvh) {
		this.gvh = gvh;
		this.name = gvh.id.getName();
		results = new Object[0];
	}
	
	/**
	 * Create a new StarLCallable with a named thread (useful for simulator debugging purposes if deadlocks are encountered)
	 * @param gvh the main global variable holder
	 * @param name the name to be given to the spawned thread
	 */
	public StarLCallable(GlobalVarHolder gvh, String name) {
		this.gvh = gvh;
		this.threadname = name;
		this.name = gvh.id.getName();
		results = new Object[0];
	}
	
	@Override
	public List<Object> call() throws Exception {
		if(threadname == null) {
			Thread.currentThread().setName("starlCallable-"+gvh.id.getName());
		} else {
			Thread.currentThread().setName(threadname + "-"+gvh.id.getName());
		}
		gvh.threadCreated(Thread.currentThread());
		List<Object> retval = callStarL();
		gvh.threadDestroyed(Thread.currentThread());
		if(retval != null)
			retval.add(0, name);
		return retval;
	}
	
	/**
	 * @return the List representation of the results variable
	 */
	protected List<Object> returnResults() {
		return new ArrayList<Object>(Arrays.asList(results));
	}
	
	/**
	 * Called by call(), code placed in callStarL will execute on its own thread.
	 * @return a List of Objects 
	 */
	public abstract List<Object> callStarL();
}
