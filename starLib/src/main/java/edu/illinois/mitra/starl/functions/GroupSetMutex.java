package edu.illinois.mitra.starl.functions;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.illinois.mitra.starl.comms.*;
import edu.illinois.mitra.starl.gvh.*;
import edu.illinois.mitra.starl.interfaces.*;
import edu.illinois.mitra.starl.objects.Common;
import edu.illinois.mitra.starl.objects.ItemPosition;
/*
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.MessageListener;
import edu.illinois.mitra.starl.interfaces.MutualExclusion;
import edu.illinois.mitra.starl.interfaces.StarLCallable;
*/

public class GroupSetMutex implements MutualExclusion, MessageListener {

	private static final String TAG = "GSMutex";
	private static final String ERR = "Critical Error";
	private GlobalVarHolder gvh;
	private int mutex_id;
	private List<String> party = null;
	private Set<Integer> sections;
	private List<RobotMessage> msgQueue = new ArrayList<RobotMessage>();
	private List<RobotMessage> toremoveQueue = new ArrayList<RobotMessage>();
	private int timeStamp;
	private boolean hasToken = false;
	private String name = null;
	
	public GroupSetMutex(GlobalVarHolder gvh, int mutex_id){
		this.gvh = gvh;
		this.mutex_id = mutex_id;
		name = gvh.id.getName();
		gvh.trace.traceEvent(TAG, mutex_id + " created", gvh.time());
		try{
			gvh.comms.addMsgListener(this, Common.MSG_GSMUTEX_REQUEST+mutex_id, Common.MSG_GSMUTEX_REPLY+mutex_id);
		} catch (RuntimeException e) {
			System.out.println("Already have a listener for MID " +  Common.MSG_GSMUTEX_REQUEST+mutex_id + " and "+Common.MSG_GSMUTEX_REPLY+mutex_id);
		}
		party= new ArrayList<String>();
		for(ItemPosition cur: gvh.gps.get_robot_Positions().getList()){
			party.add(0, cur.name);
		}
		if(party.contains(name)){
			timeStamp = party.indexOf(name);
		}
		else{
			sections = null;
		}
	}
	
	@Override
	public void cancel() {
		gvh.trace.traceEvent(TAG,  "Cancelled", gvh.time());
		gvh.comms.removeMsgListener(Common.MSG_GSMUTEX_REQUEST+mutex_id);
		gvh.comms.removeMsgListener(Common.MSG_GSMUTEX_REPLY+mutex_id);
	}

	@Override
	public void setGroup(List<String> party_in) {
		party= new ArrayList<String>();
		party.addAll(party_in);
		if(party_in.contains(name)){
			timeStamp = party_in.indexOf(name);
		}
		else{
			sections = null;
		}
	}

	@Override
	public void requestEntry(int id) {
		sections = new HashSet<Integer>();
		sections.add(new Integer(id));
		requestEntry_helper();
	}

	@Override
	public void requestEntry(Set<Integer> ids) {
		sections = new HashSet<Integer>();
		sections.addAll(ids);
		requestEntry_helper();
	}
	
	private void requestEntry_helper(){
		checkQueue();
		String[] section_string = new String[sections.size()+2];
		int temp = 0;
		for (Integer s : sections) {
			section_string[temp] = s.toString();
			temp++;
		}
		section_string[section_string.length-2] = String.valueOf(mutex_id);
		//System.out.println(String.valueOf(mutex_id));
		section_string[section_string.length-1] = String.valueOf(timeStamp);
		//attach the timeStamp at the end of the message
		MessageContents sections_msg = new MessageContents(section_string);
		RobotMessage request = new RobotMessage("ALL", name, Common.MSG_GSMUTEX_REQUEST+mutex_id, sections_msg);
		gvh.comms.addOutgoingMessage(request);
		party.remove(name);
		gvh.trace.traceEvent(TAG,  "Request Sent", gvh.time());
		gvh.log.d(TAG, "Request Sent to party");
	}

	@Override
	public boolean clearToEnter(int id) {
		if(!sections.contains(id))
			return false;
		if(party.size() == 0){
			hasToken = true;
		}
		return hasToken;
	}

	@Override
	public boolean clearToEnter(Set<Integer> ids) {
		if(!sections.containsAll(ids)){
			return false;
		}
		if(party.size() == 0){
			hasToken = true;
		}
		return hasToken;
	}

	@Override
	public void exit(int id) {
		if(sections.contains(id)){
			sections.remove(id);
			exitHelper(new String(String.valueOf(id)));
		}
	}

	@Override
	public void exit(Set<Integer> ids) {
		boolean release = false;
		StringBuffer ids_string = new StringBuffer();
		for (Integer id : ids) {
			if(sections.contains(id)){
				sections.remove(id);
				ids_string.append(id);
				release = true;
			}
		}
		if(release){
			exitHelper("exit " + ids_string.toString());
		}
	}

	@Override
	public void exitAll() {
		if(!sections.isEmpty()){
			sections.clear();
			exitHelper("exit all");
		}
	}
	
	private void exitHelper(String msg){
		checkQueue();
		gvh.log.d(TAG, msg);
		if(sections.isEmpty()){
			hasToken = false;
		}
	}

	@Override
	public void messageReceived(RobotMessage m) {
//		System.out.println(m.toString()+ " mutex_id : "+ mutex_id);
		if(m.getTo().equals(name) || m.getTo().equals("ALL")){
			if(m.getMID() == Common.MSG_GSMUTEX_REQUEST+mutex_id){
				String id = m.getFrom();
				MessageContents msg_content = m.getContents();
				List<String> R_request = new ArrayList<String>(msg_content.getContents());
				int tStamp = Integer.parseInt(R_request.remove(R_request.size()-1));
				int cur_id = Integer.parseInt(R_request.remove(R_request.size()-1));
				//return without doing anything if the mutex id is different
				if(cur_id != mutex_id){
					return;
				}
				//get the sections and the timeStamp
				if(sections != null && sections.size() != 0 && !hasToken){
					boolean intersect = checkIntersect(R_request);
					if(intersect && ((tStamp>timeStamp) || ((tStamp == timeStamp) && id.compareTo(name) > 0)))
						//if(intersect and (m.timeStamp,m.id)>(timeStamp,id))
						QueueMSG(m);
					else
						replyToRequest(m);
				}
				if(hasToken){
					boolean intersect = checkIntersect(R_request);
					if(intersect)
						QueueMSG(m);
					else
						replyToRequest(m);
				}
				if(sections == null || sections.size() == 0){
					//has not started requesting or has exited all sections
					replyToRequest(m);
				}
				else{
					//not requested yet, queue message
					QueueMSG(m);
				}
				return;
				
			}
			if(m.getMID() == Common.MSG_GSMUTEX_REPLY+mutex_id && Integer.parseInt(m.getContents(0)) == mutex_id){
                if(party.contains(m.getFrom())){
                    party.remove(m.getFrom());
                }
                else{
                    QueueMSG(m);
                }
                return;
			}
		}
	}
	
	private void checkQueue() {
		if(!msgQueue.isEmpty()){
			for (RobotMessage temp : msgQueue) {
			    messageReceived(temp);
			}
		}
		while(!toremoveQueue.isEmpty()){
			RobotMessage temp2 = toremoveQueue.remove(0);
			msgQueue.remove(temp2);
		}
	}
	
	private void replyToRequest(RobotMessage m2) {
		//System.out.println(name + " replying to "+m2.getFrom() + " at Stage " + stage);
		if(msgQueue.contains(m2)){
			toremoveQueue.add(m2);
			//System.out.println("adding reply to "+m2);
		}
		String id = m2.getFrom();
		MessageContents mutex_id_msg = new MessageContents(String.valueOf(mutex_id));
		RobotMessage request = new RobotMessage(id, name, Common.MSG_GSMUTEX_REPLY+mutex_id, mutex_id_msg);
		gvh.comms.addOutgoingMessage(request);
	}

	private void QueueMSG(RobotMessage m) {
		if(msgQueue.contains(m)){
			//System.out.println(name + " queueing "+m);
			return;
		}
		//Queue the message
		msgQueue.add(m);
	}

	private boolean checkIntersect(List<String> R_request){
		if(sections == null || sections.size() == 0){
			return false;
		}
		boolean intersect = false;
		for(int i = 0; i<R_request.size(); i++){
			if(sections.contains(Integer.parseInt(R_request.get(i)))){
				intersect = true;
			}
		}
		return intersect;
	}
}
