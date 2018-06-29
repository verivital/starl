package edu.illinois.mitra.starl.objects;

import java.util.*;

import edu.illinois.mitra.starl.motion.RRTNode;

/**
 * The obstacle list is defined here
 * It contains methods that are useful for path planning
 * @author Yixiao Lin
 * @version 1.0
 */

public class ObstacleList {
	public Vector<Obstacles> ObList;
	public int detect_Precision;
	public int de_Radius;
	private long lastUpdateTime;

	/**
    *
    * @param Oblist
    */
	public ObstacleList(Vector<Obstacles> Oblist) {
		this.ObList = (Vector<Obstacles>)Oblist.clone();
		lastUpdateTime = System.currentTimeMillis();
	}

    /**
    *
    */
	public ObstacleList(){
		this.ObList = new Vector<Obstacles>(3,2);
		lastUpdateTime = System.currentTimeMillis();
	}


	public ObstacleList clone(){
		ObstacleList toReturn = new ObstacleList(ObList);
		toReturn.de_Radius = this.de_Radius;
		toReturn.detect_Precision = this.detect_Precision;
		toReturn.lastUpdateTime = this.lastUpdateTime;
		return toReturn;

	}


    /**
     * check if the line alone destination and current has any intersection with any obstacles
     */
	public boolean badPath(ItemPosition destination, ItemPosition current){
		boolean check = false;
		for(int i=0; i< ObList.size() && ObList.elementAt(i) != null; i++){
			if(ObList.elementAt(i).checkCross(destination, current)){
				return true;
			}
		}
		return false;
	}

	/**
	 *
	 * @param Oblist
	 */
	public void addObstacles(Vector<Obstacles> Oblist) {
		this.ObList.addAll((Vector<Obstacles>)Oblist.clone());
		this.updateObs();
	}

    /**
     *
     * @param o
     */
    public void addObstacle(Obstacles o) {
        this.ObList.add(o);
        this.updateObs();
    }

	/**
	* This method is used for checking if the line alone destination and current has any intersection with any obstacles
	*/
	public boolean badPath(RRTNode destinationNode, RRTNode currentNode){
		ItemPosition destination = new ItemPosition("NodeToIPDes", destinationNode.position.x(), destinationNode.position.y(), 0);
		ItemPosition current = new ItemPosition("NodeToIPCurrt", currentNode.position.x(), currentNode.position.y(), 0);
		boolean check = false;
		for(int i=0; i< ObList.size() && ObList.elementAt(i) != null; i++){
			if(ObList.elementAt(i).checkCross(destination, current)){
				return true;
			}
		}
		return false;
	}

	/**
	 * check if the point is reachable by robot
	 * @param destination
	 * @param radius
	 * @return boolean
	 *
	 *
	 */
	public boolean validstarts(ItemPosition destination, double radius){
		if(destination == null){
			return false;
		}
		if(ObList == null){
			return true;
		}
		boolean check = true;
		for(int i=0; i< ObList.size(); i++){
			if(ObList.elementAt(i) != null){
				check = check && ObList.elementAt(i).validItemPos(destination, radius);
			}
			else{
				break;
			}
		}
		return check;
	}

	/**
	 * return true if the path specified by two RRTNode has a line such that every point alone the line is reachable by robots.
	 * It checks if two line segments smallest distance is bigger than radius
	 * For example, line AB and CD, the shortest distance is minimum of A to CD, B to CD, C to AB, D to AB
	 * @param destinationNode
	 * @param currentNode
	 * @param Radius
	 * @return
	 */

	public boolean validPath(RRTNode destinationNode, RRTNode currentNode,  int Radius){
		if(destinationNode == null){
			//System.out.println("Added node was null");
			return false;
		}
		if(ObList == null){
			return true;
		}
		if(badPath(destinationNode, currentNode)) {
			//System.out.println("Added node had bad path");
			return false;
		}
		else{
			for(int i=0; i< ObList.size() && ObList.elementAt(i) != null; i++){
                double minDist = ObList.elementAt(i).findMinDist(destinationNode, currentNode) - 20;
                if(minDist < Radius){
                    //System.out.println("Added node was too close to obstacle, minDist: " + minDist + " Radius: " + Radius);
					return false;
                }
			}
			return true;
		}
	}


	/**
	 * methods for hidden or time vise obstacles
	 *
	 * Removes obstacles from ObList if they are too old
	 */
	public void updateObs(){
		long duration = System.currentTimeMillis() - lastUpdateTime;
		synchronized (ObList){
		for(int i=0; i< ObList.size(); i++){
			if(ObList.elementAt(i) != null){
				if(ObList.elementAt(i).timeFrame == 0){
					ObList.remove(i);
					updateObs();
					break;
				}
				else{
					if(ObList.elementAt(i).timeFrame > 0){
						ObList.elementAt(i).timeFrame -= duration;
						if(ObList.elementAt(i).timeFrame <0)
							ObList.elementAt(i).timeFrame = 0;
					}
				}
			}
			else
			break;
		}
		}
	}

	/**
     * download method to provide robots with informations of unhidden obstacles
     * sever as an selected deep copy
     *
     * @return
     */
	public ObstacleList downloadObs() {
		if(ObList == null)
			return null;

		ObstacleList obsList = new ObstacleList();
		for(int i = 0; i< ObList.size(); i++){
			Obstacles temp = new Obstacles(ObList.get(i));
			if(!ObList.get(i).hidden)
			obsList.ObList.addElement(temp);
		}
		obsList.de_Radius = de_Radius;
		obsList.detect_Precision = detect_Precision;
		return obsList;
	}

    /**
     * change the obstacle map to grid wise representation of obstacles, detect_Precision is the length of the grid, 1 as minimum
     */
	public void Gridfy() {
		for(int i=0; i< ObList.size(); i++){
			if(ObList.elementAt(i) != null){
				ObList.elementAt(i).ToGrid(detect_Precision);
			}
		}
	}

	/**
	 * add the detected obstacle to the current map
	 * Please avoid using this method on the gvh obstacle map, it will cause all other robot to detect this obstacle as well
	 *
	 * @param blocker
	 */
	public void detected(ItemPosition blocker){
		boolean uncontained = true;
		for(int i=0; i< ObList.size(); i++){
			if(ObList.elementAt(i) != null){
				uncontained = uncontained && ObList.elementAt(i).validItemPos(blocker);
			}

		}
		if(uncontained){
			Obstacles newObs = new Obstacles(blocker.x(), blocker.y());
			ObList.addElement(newObs);
			ObList.lastElement().ToGrid(detect_Precision * de_Radius);
		}
	}

	/**
     *
     * @param robotPos
     * @param radius
     */
	public void remove(ItemPosition robotPos, double radius){
		if(robotPos == null)
			return;
		if(ObList == null)
			return;

		for(int i=0; i< ObList.size(); i++){
			if(ObList.elementAt(i) != null){
				if(!ObList.elementAt(i).validItemPos(robotPos, radius)){
					ObList.remove(i);
				}
			}
			else
			break;
		}
		return;
	}

}
