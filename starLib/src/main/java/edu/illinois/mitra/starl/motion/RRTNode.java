package edu.illinois.mitra.starl.motion;

import java.util.Stack;

import edu.illinois.mitra.starl.objects.ItemPosition;
import edu.illinois.mitra.starl.objects.ObstacleList;
import edu.illinois.mitra.starl.objects.Point3i;
import edu.wlu.cs.levy.CG.KDTree;
import edu.wlu.cs.levy.CG.KeySizeException;

/**
 * This implements RRT path finding algorithm using kd tree
 * 
 * @author Yixiao Lin
 * @version 1.0
 */

public class RRTNode {
	public Point3i position;
	public RRTNode parent;
	public static RRTNode stopNode;
	public KDTree<RRTNode> kd;
//	public LinkedList<ItemPosition> pathList = new LinkedList<ItemPosition>();

	public double [] getValue(){
		double [] toReturn = {position.getX(), position.getY()};
		return toReturn;
	}

	public RRTNode(){
		position = new Point3i();
		parent = null;
	}
	
	public RRTNode(int x, int y){
		position = new Point3i(x, y);
		parent = null;
	}

	public RRTNode(RRTNode copy){
		position = new Point3i(copy.position);
		parent = copy.parent;
	}

	/**
	 * methods to find the route
	 if find a path, return a midway Point3i stack
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
    	double [] root = {position.getX(), position.getY()};
    	final RRTNode rootNode = new RRTNode(position.getX(), position.getY());
    	final RRTNode destNode = new RRTNode(destination.getX(), destination.getY());
    	
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
			//System.out.println("Adding node (" + addedNode.position.getX + ", " + addedNode.position.getY + ")");
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
    		//random a sample Point3i in the valid set of space
    		boolean validRandom = false;
    		int xRandom = 0;
    		int yRandom = 0;
    		ItemPosition sampledPos = new ItemPosition("rand",xRandom, yRandom, 0);
    		while(!validRandom){
				xRandom = (int) Math.round((Math.random() * ((xUpper - xLower))));
				yRandom = (int) Math.round((Math.random() * ((yUpper - yLower))));
				sampledPos.setPos(xRandom + xLower, yRandom + yLower);
				validRandom = ((sampledPos.getX() >= xLower && sampledPos.getX() <= xUpper) && (sampledPos.getY() >= yLower && sampledPos.getY() <= yUpper));
				validRandom = validRandom && obsList.validstarts(sampledPos, radius);
				if(validRandom){
                    // added a check to see if sampledPos is already in tree
                    boolean notInTree = true;
                    RRTNode possibleNode = new RRTNode(sampledPos.getX(), sampledPos.getY());
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
    		RRTNode sampledNode = new RRTNode(sampledPos.getX(), sampledPos.getY());
    		// with a valid random sampled Point3i, we find it's nearest neighbor in the tree, set it as current Node
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
			ItemPosition ToGo= new ItemPosition("midpoint", curNode.position.getX(), curNode.position.getY());
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
     * It changes sampledNode to some Point3i alone the line of sampledNode and currentNode so that no obstacles are in the middle
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
			toggleNode.position = new Point3i((int)((toggleNode.position.getX() + currentNode.position.getX())/1.5),
					(int)((toggleNode.position.getY() + currentNode.position.getY())/1.5));
			tries++;
		}
		//return currentNode if toggle failed
		// TODO: remove magic number
		if(tries >= 19)
			return null;
		else
			return toggleNode;
	}
}



