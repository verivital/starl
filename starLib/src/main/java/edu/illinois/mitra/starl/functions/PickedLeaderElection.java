//Written By Lucas Buccafusca
//6-15-2012
//PickedLeaderElection takes the 'first' robot and makes it the leader
// The 'first' robot is the one with the alphabetical first name

package edu.illinois.mitra.starl.functions;

import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledThreadPoolExecutor;

import edu.illinois.mitra.starl.comms.MessageContents;
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.MessageListener;
import edu.illinois.mitra.starl.interfaces.StarLCallable;
import edu.illinois.mitra.starl.objects.Common;

/**
 * Elects a leader. All agents broadcast a random integer, the robot with the largest number wins the election.
 * Ties are broken by comparing agent identifiers lexicographically (word of the day). To ensure proper operation,
 * robots should be synchronized before electing!
 * 
 * @author Adam Zimmerman
 * @version 1.0
 *
 */
public class PickedLeaderElection extends StarLCallable implements LeaderElection, MessageListener {

	private static final String TAG = "RandomLeaderElection";
	private static final String ERR = "Critical Error";
	private int nodes = 0;
	private static int MAX_WAIT_TIME = 5000;
	
	private SortedSet<Ballot> ballots = new TreeSet<Ballot>();
	private Set<String> receivedFrom = new HashSet<String>();
	
	private String announcedLeader = null;
	
	private ExecutorService executor = new ScheduledThreadPoolExecutor(1);

	private Future<List<Object>> elected;
	
	public PickedLeaderElection(GlobalVarHolder gvh) {
		super(gvh,"RandomLeaderElection");
		results = new String[2];
		nodes = gvh.id.getParticipants().size();
		gvh.trace.traceEvent(TAG, "Created", gvh.time());
		registerListeners();
	}
			
	@Override
	public List<Object> callStarL() {
		gvh.trace.traceEvent(TAG, "Beginning Election", gvh.time());
		gvh.log.d(TAG, "Beginning election...");
		nodes = gvh.id.getParticipants().size();
		boolean error = false;

		// Generate a random number
		Random rand = new Random();
		int myNum = rand.nextInt(1000);
		receivedFrom.add(name);
		ballots.add(new Ballot(name, myNum, gvh.id.getIdNumber()));

		gvh.trace.traceVariable(TAG, "myNum", myNum, gvh.time());
		gvh.log.i(TAG, "My number is " + myNum);
		
		// Broadcast
		RobotMessage bcast = new RobotMessage("ALL", name, Common.MSG_RANDLEADERELECT, new MessageContents(Integer.toString(myNum), Integer.toString(gvh.id.getIdNumber())));
		gvh.comms.addOutgoingMessage(bcast);
		
		// Wait to receive MSG_LEADERELECT messages
		Long endTime = gvh.time()+MAX_WAIT_TIME;
		gvh.trace.traceEvent(TAG, "Waiting for MSG_LEADERELECT messages", gvh.time());
		while(!error && receivedFrom.size() < nodes) {
			if(gvh.time() >= endTime) {
				gvh.trace.traceEvent(TAG, "Waited timed out", gvh.time());
				gvh.log.e(TAG, "Waited too long!");
				
				Set<String> ptc = new HashSet<String>(gvh.id.getParticipants());
				ptc.removeAll(receivedFrom);
				System.out.println(name + " has waited too long to receive election messages. Have only received " + receivedFrom.size() + "\n\t\tWe're missing from " + ptc.toString());
				if(!receivedFrom.contains(name)) System.out.println("!!!!!" + name + " IS MISSING A VALUE FROM ITSELF??");
				error = true;
			}
			gvh.sleep(10);
		}
		
		gvh.log.d(TAG, "Received all numbers, determining leader.");
		// Determine the leader
		Object[] botarray;
		String leader=null;
		int leaderID = -1;
		if(!error) {
			gvh.log.d(TAG, "No errors, determining leader now.");
			// Retrieve all names and take the one whose name comes first (in order of the alphabet)
			botarray =gvh.id.getParticipants().toArray();
			leader = ballots.first().candidate;
			leaderID = ballots.first().idNum;
			//leaderID = botarray[0].id;
			
			gvh.trace.traceEvent(TAG, "Determined leader", leader, gvh.time());
			
			// Have determined a leader, broadcast the result
			RobotMessage bcast_leader = new RobotMessage("ALL", name, Common.MSG_RANDLEADERELECT_ANNOUNCE, new MessageContents(leader));
			gvh.comms.addOutgoingMessage(bcast_leader);
			gvh.trace.traceEvent(TAG, "Notified all of leader", gvh.time());

		}		
		if(error) {
			gvh.log.d(TAG, "An error occurred (waited too long?) must wait to receive announcement broadcasts.");
			// Receive any MSG_LEADERELECT_ANNOUNCE messages, accept whoever they elect as leader
			endTime = gvh.time()+MAX_WAIT_TIME;
			gvh.trace.traceEvent(TAG, "Waiting for MSG_LEADERELECT_ANNOUNCE messages", gvh.time());
			while(announcedLeader == null) {
				if(gvh.time() > endTime) {
					gvh.trace.traceEvent(TAG, "Waited timed out, leader election failed", gvh.time());
					gvh.log.e(TAG, "Leader election failed!");
					results[0] = "ERROR";
					return returnResults();
				}
				gvh.sleep(10);
			}
			leader = announcedLeader;
		}

		gvh.log.i(TAG, "Elected leader: " + leader);
		gvh.trace.traceEvent(TAG, "Elected leader", leader, gvh.time());
		results[0] = leader;
		results[1] = Integer.toString(leaderID);
		return returnResults();
	}
	
	public void messageReceived(RobotMessage m) {
		gvh.log.i(TAG, "Received a message from " + m.getFrom() + ": " + m.getContents(0));
		switch(m.getMID()) {
		case Common.MSG_RANDLEADERELECT:
			String from = m.getFrom();
			if(receivedFrom.contains(from)) {
				gvh.log.e(TAG, "Received from " + from + " twice!");
			} else {
				int val = Integer.parseInt(m.getContents(0));
				int id = Integer.parseInt(m.getContents(1));

				ballots.add(new Ballot(from, val, id));
				receivedFrom.add(from);
				gvh.log.i(TAG, "Received " + receivedFrom.size());
				if(receivedFrom.size() == nodes) {
					gvh.log.i(TAG, "READY TO ELECT A LEADER!");
				}
			}
			gvh.trace.traceEvent(TAG, "Received MSG_RANDLEADERELECT message", m, gvh.time());
			break;
			
		case Common.MSG_RANDLEADERELECT_ANNOUNCE:
			announcedLeader = m.getContents(0);
			gvh.trace.traceEvent(TAG, "Received MSG_RANDLEADERELECT_ANNOUNCE message", announcedLeader, gvh.time());
			break;
		}
	}
	
	public void elect() {
		elected = executor.submit(this);
	}

	@Override
	public int getLeaderID(){
		if(elected.isDone()) {
			try {
				return Integer.parseInt((String)elected.get().get(2));
			} catch (InterruptedException e) {
				e.printStackTrace();
			} catch (ExecutionException e) {
				e.printStackTrace();
			}
		}
		return -1;
	}

	@Override
	public String getLeader() {
		if(elected.isDone()) {
			try {
				return (String)(elected.get().get(1));
			} catch (InterruptedException e) {
				e.printStackTrace();
			} catch (ExecutionException e) {
				e.printStackTrace();
			}
			return "ERROR";
		}
		return null;
	}
	
	private void registerListeners() {
		gvh.comms.addMsgListener(this, Common.MSG_RANDLEADERELECT, Common.MSG_RANDLEADERELECT_ANNOUNCE);
	}

	private void unregisterListeners() {
		gvh.comms.removeMsgListener(Common.MSG_RANDLEADERELECT);
		gvh.comms.removeMsgListener(Common.MSG_RANDLEADERELECT_ANNOUNCE);		
	}
	
	
	private class Ballot implements Comparable<Ballot> {
		public String candidate;
		public int value;
		public int idNum;

		
		public Ballot(String candidate, int value, int id) {
			this.candidate = candidate;
			this.value = value;
			this.idNum = id;
		}
		
		public String toString() {
			return candidate;
		}


		@Override
		public int compareTo(Ballot other) {
			if(other.value == this.value) {
				return candidate.compareTo(other.candidate);
			}
			return value - other.value;
		}		
	}
	
	@Override
	public void cancel() {
		executor.shutdownNow();
		unregisterListeners();
		gvh.trace.traceEvent(TAG, "Cancelled", gvh.time());
	}
}
