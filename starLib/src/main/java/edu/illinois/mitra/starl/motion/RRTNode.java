package edu.illinois.mitra.starl.motion;

import java.util.Stack;

import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3d;
import edu.wlu.cs.levy.CG.KDTree;
import edu.wlu.cs.levy.CG.KeySizeException;

/**
 * This implements RRT path finding algorithm using kd tree
 * 
 * @author Yixiao Lin
 * @version 1.0
 */

public class RRTNode {
	public Point3d position = new Point3d();
	public RRTNode parent;
	public static RRTNode stopNode;
	public KDTree<RRTNode> kd;
//	public LinkedList<ItemPosition> pathList = new LinkedList<ItemPosition>();

	public double [] getValue(){
		double [] toReturn = {position.x(), position.y()};
		return toReturn;
	}

	public RRTNode(){
		position.x(0);
		position.y(0);
		parent = null;
	}
	
	public RRTNode(int x, int y){
		position.x(x);
		position.y(y);
		parent = null;
	}

	public RRTNode(RRTNode copy){
		position.x(copy.position.x());
		position.y(copy.position.y());
		parent = copy.parent;
	}

	/**
	 * methods to find the route
	 if find a path, return a midway Point3d stack
	 if can not find a path, return null
	 remember to handle the null stack when writing apps using RRT path planning
	 the obstacle list will be modified to remove any obstacle that is inside a robot
	 * @param destination
	 * @param K
	 * @param obsList
	 * @param xLower
	 * @param xUpper
	 * @param yLower
	 * @param yUpper
	 * @param RobotPos
	 * @param radius
	 * @return
	 */
	
    	//initialize a kd tree;

	public Stack<ItemPosition> findRoute(ItemPosition destination, int K, ObstacleList obsList, int xLower, int xUpper, int yLower, int yUpper, ItemPosition RobotPos, int radius) {
		//TODO: add Steer here
		//initialize a kd tree;
		//obsList.remove(RobotPos, 0.9*Radius);
		if(xLower > xUpper || yLower> yUpper){
			System.err.println("Lower bound must be smaller or equal to than upper bound");
			return null;
		}
    	kd = new KDTree<RRTNode>(2);
    	double [] root = {position.x(), position.y()};
    	final RRTNode rootNode = new RRTNode(position.x(), position.y());
    	final RRTNode destNode = new RRTNode(destination.x(), destination.y());
    	
    	try{
    		kd.insert(root, rootNode);
    	}
    	catch(Exception e){
    		System.err.println(e);
    	}	
    	
    	RRTNode currentNode = new RRTNode(rootNode);
    	RRTNode addedNode = new RRTNode(rootNode);
    //for(i< k)  keep finding	
    	for(int i = 0; i<K; i++){
    	//if can go from current to destination, meaning path found, add destinationNode to final, stop looping.
			//System.out.println("Adding node (" + addedNode.position.x + ", " + addedNode.position.y + ")");
			if(obsList.validPath(addedNode, destNode, radius)){
    			destNode.parent = addedNode;
    			stopNode = destNode;
    			try{	
    			kd.insert(destNode.getValue(), destNode);
    			}
        		catch (Exception e) {
        		    System.err.println(e);
        		}
    			//System.out.println("Path found!");
    			break;
    		} else {
				//System.out.println("Added node was invalid.");
			}
    		//not find yet, keep exploring
    		//random a sample Point3d in the valid set of space
    		boolean validRandom = false;
    		int xRandom = 0;
    		int yRandom = 0;
    		ItemPosition sampledPos = new ItemPosition("rand",xRandom, yRandom, 0);
    		while(!validRandom){
				xRandom = (int) Math.round((Math.random() * ((xUpper - xLower))));
				yRandom = (int) Math.round((Math.random() * ((yUpper - yLower))));
				sampledPos.x(xRandom + xLower);
				sampledPos.y(yRandom + yLower);
				validRandom = ((sampledPos.x() >= xLower && sampledPos.x() <= xUpper) && (sampledPos.y() >= yLower && sampledPos.y() <= yUpper));
				validRandom = validRandom && obsList.validstarts(sampledPos, radius);
				if(validRandom){
                    // added a check to see if sampledPos is already in tree
                    boolean notInTree = true;
                    RRTNode possibleNode = new RRTNode(sampledPos.x(), sampledPos.y());
                    try {
                        if(kd.search(possibleNode.getValue()) != null) {
                            notInTree = false;
                        }
                    } catch (KeySizeException e) {
                        e.printStackTrace();
                    }
                    validRandom = (validRandom && notInTree);
                }
			}
    		RRTNode sampledNode = new RRTNode(sampledPos.x(), sampledPos.y());
    		// with a valid random sampled Point3d, we find it's nearest neighbor in the tree, set it as current Node
    		try{
    		currentNode = kd.nearest(sampledNode.getValue());
    		}
    		catch (Exception e) {
    		    System.err.println(e);
    		}
			sampledNode = toggle(currentNode, sampledNode, obsList, radius);
    		//check if toggle failed
    		//if not failed, insert the new node to the tree
    		if(sampledNode != null){
    			sampledNode.parent = currentNode;
    			try{
    	    		kd.insert(sampledNode.getValue(), sampledNode);
    	    		}
    	    		catch (Exception e) {
    	    		    System.err.println(e);
    	    		}
    			//set currentNode as newest node added, so we can check if we can reach the destination
    			addedNode = sampledNode;
    			//
    		}
    	}
    	stopNode = addedNode;
	
    	//after searching, we update the path to a stack

      	RRTNode curNode = destNode;  	
		Stack<ItemPosition> pathStack= new Stack<ItemPosition>();
		while(curNode != null){
			ItemPosition ToGo= new ItemPosition("midpoint", curNode.position.x(), curNode.position.y());
			pathStack.push(ToGo);
			curNode = curNode.parent;
		}
    	
    	if(destNode.parent == null){
			System.out.println("Path Not found! Tree size: " + kd.size());
    		return(null);
    	}
    	else{
    		stopNode = destNode;
    		return pathStack;
    	}
    }

    /**
     * toggle function deals with constrains by the environment as well as robot systems.
     * It changes sampledNode to some Point3d alone the line of sampledNode and currentNode so that no obstacles are in the middle
     * In other words, it changes sampledNode to somewhere alone the line where robot can reach
     *
     * TODO: we can add robot system constraints later
     * 
     * @param currentNode
     * @param sampledNode
     * @param obsList
     * @param radius
     * @return
     */
	private RRTNode toggle(RRTNode currentNode, RRTNode sampledNode, ObstacleList obsList, int radius) {
		RRTNode toggleNode = new RRTNode(sampledNode);
		int tries = 0;
		// try 20 times, which will shorten it to 0.00317 times the original path length
		// smaller tries might make integer casting loop forever
		while((!obsList.validPath(toggleNode, currentNode, radius)) && (tries < 20)){
			//move 1/4 toward current
			toggleNode.position.x((int) ((toggleNode.position.x() + currentNode.position.x())/(1.5)));
			toggleNode.position.y((int) ((toggleNode.position.y() + currentNode.position.y())/(1.5)));
			tries ++;
		}
		//return currentNode if toggle failed
		// TODO: remove magic number
		if(tries >= 19)
			return null;
		else
			return toggleNode;
	}
}



