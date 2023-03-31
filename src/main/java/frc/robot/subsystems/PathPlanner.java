// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;

/** Add your docs here. */
public class PathPlanner {

	private Graph graph;

	public PathPlanner(){
		graph = new Graph();
		graph.addNode(new Node(-3.87, 2.9, -1.21)); //STOW 0
		graph.addNode(new Node(-2.6, 2.8, -1.2)); //STOW_INTERMEDIATE 1
		graph.addNode(new Node(-1.483, 3.570, -0.337)); //CUBE_TOP 2
		graph.addNode(new Node(-1.72, 3.8, -0.26)); //CUBE_MID 3
		graph.addNode(new Node(-2.0, 3.19, -0.77)); //CUBE_TOP_INTERMEDIATE 4
		graph.addNode(new Node(-1.209, 3.224, 1.137)); //CONE_TOP 5
		graph.addNode(new Node(-1.72, 3.545, 1.189)); //CONE_MID 6
		graph.addNode(new Node(-2.0, 3.0, 0.0)); //CONE_TOP_INTERMEDIATE 7
		graph.addNode(new Node(-2.45, 3.47, -0.33)); //DOUBLE_SUB_CUBE 8  
		graph.addNode(new Node(-2.11, 3.1, 1.41)); //DOUBLE_SUB_CONE 9
		graph.addNode(new Node(-2.6, 2.8, -1.2)); //DOUBLE_SUB_CONE_INTERMEDIATE 10
		graph.addNode(new Node(-2.6, 3.2, -0.6)); //DOUBLE_SUB_CUBE_INTERMEDIATE 11
		graph.addNode(new Node(-1.19, 5.38, -1.21)); //GROUND_CUBE 12
		graph.addNode(new Node(-1.73, 4.59, 0.67)); //GROUND_CONE 13
		graph.addNode(new Node(-1.78, 3.98, -1.19)); //GROUND_INTERMEDIATE 14
		graph.addNode(new Node(-1.458, 3.3565, 0.3665)); //CONE_TOP to CUBE_TOP INTERMEDIATE 15
		graph.addNode(new Node(-2.0719, 4.7582, 0.8069)); //STOW_ALT 16
		graph.addNode(new Node(-1.63095, 5.0691, -0.20155)); // CUBE_GROUND to STOW_ALT INTERMEDIATE 17
		graph.addNode(new Node(-1.77745, 3.7791, -1.00345)); // CUBE_TOP to STOW_ALT INTERMEDIATE 18
		graph.addNode(new Node(-2.26095, 4.1141, -0.56845)); // DOUBLE_SUB_CUBE to STOW_ALT INTERMEDIATE 19
		graph.addNode(new Node(-1.815, 3.5105, 0.436)); //CUBE_MID to INTERMEDIATE_MID 20
		
		//previous connections
		graph.addEdge(4, 2);
		graph.addEdge(0, 4);
		graph.addEdge(10, 0);
		graph.addEdge(10, 9);
		graph.addEdge(7, 5);
		graph.addEdge(0,7);
		graph.addEdge(0,11);
		graph.addEdge(11, 8);
		graph.addEdge(0,1);
		graph.addEdge(1, 3);
		graph.addEdge(1, 6);
		graph.addEdge(1, 14);
		graph.addEdge(14, 12);
		graph.addEdge(14, 13);

		//new connections
		graph.addEdge(4, 7);
		graph.addEdge(4, 1);
		graph.addEdge(7, 1);
		graph.addEdge(10, 11);
		graph.addEdge(11, 1);
		graph.addEdge(10, 1);
		graph.addEdge(4, 10);

		graph.addEdge(2,15);
		graph.addEdge(15,5);
		graph.addEdge(3,20);
		graph.addEdge(6,20);
		graph.addEdge(1,16);
		graph.addEdge(4,11);
		graph.addEdge(7,10);
		graph.addEdge(7,11);
		graph.addEdge(1,16);
		graph.addEdge(12,17);
		graph.addEdge(13,17);
		graph.addEdge(17,16);
		graph.addEdge(9,19);
		graph.addEdge(8,19);
		graph.addEdge(19,16);
		graph.addEdge(2,18);
		graph.addEdge(5,18);
		graph.addEdge(18,16);
		graph.addEdge(20,18);
	









		


	}

	public class Node{
		public double shoulder;
		public double elbow;
		public double wrist;

		private boolean explored = false;  // For a given plan has this node been explored already?
		private int parent = -1;		   // What was the parent of this node in the exploration?
		private double distance = Double.MAX_VALUE; // Distance along path at this node so far.

		public Node(double shoulder, double elbow, double wrist){
			this.shoulder = shoulder;
			this.elbow = elbow;
			this.wrist = wrist;
		}

		public void clearPlan() {
			parent = -1;
			explored = false;
			distance = Double.MAX_VALUE;
		}

		public boolean isExplored() {
			return explored;
		}

		public int getParent() {
			return parent;
		}

		public double getDistance() {
			return distance;
		}

		public void markExplored() {
			explored = true;
		}

		public void setParent(int p) {
			parent = p;
		}

		public void setDistance(double d) {
			distance = d;
		}

		// Compute distance from a node to another node:
		public double distance(Node n) {
			return distance(n.shoulder, n.elbow, n.wrist);
		}

		// Compute distance from a node to joint values for finding nearest.
		public double distance(double s, double e, double w) {
			return Math.sqrt(Math.pow((shoulder -s), 2) + Math.pow((elbow - e), 2) + Math.pow((wrist - w), 2));	
		}
	}

	public class Graph{
		public ArrayList<Node> nodes;					// List of all nodes in the graph.
		public ArrayList<ArrayList<Integer>> adj;		// Set of edges in the graph as an array of edges at each node (a sparse adjacency matrix)

		public Graph(){
			nodes = new ArrayList<Node>();
			adj = new ArrayList<ArrayList<Integer>>();
		}

		public void addNode(Node node){
			node.clearPlan();
			nodes.add(node); // Add the node
			adj.add(new ArrayList<Integer>()); // Add the adjacency list entry for the node.
		}

		public void addEdge(int a, int b){
			// The adjacency matrix must be symmetric since the graph is not directed. We can go between nodes in either direction.
			adj.get(a).add(b);
			adj.get(b).add(a);
		}

		public int findClosestNode(double shoulder, double elbow, double wrist){
			double bestDistnce = Double.MAX_VALUE;
			int bestIdx = 0;
			for(int i = 0; i < nodes.size(); i++) {
				double distance = nodes.get(i).distance(shoulder, elbow, wrist);
				if(distance < bestDistnce){
					bestDistnce = distance;
					bestIdx = i;
				}
			}
			return bestIdx;
		}


		// Implementation of basic depth-first search for a path in the graph.
		// This stores a path in the graph nodes to recover with recoverPath() function.
		// It returns true if it finds the end, else false.
		public boolean findPath(int startIdx, int endIdx){
			
			// First we need to go through our graph and clear previous path out of all our nodes:
			for (int nidx = 0; nidx < nodes.size(); nidx++) {
				nodes.get(nidx).clearPlan();
			}

			// Now we're going to make an array list and use it as our search queue for DFS:
			ArrayList<Integer> queue = new ArrayList<Integer>();

			// Prime the pump by putting the first node into the queue with zero distance and no parent:
			queue.add(startIdx);
			nodes.get(startIdx).distance = 0; // We're starting here so distance is zero
			nodes.get(startIdx).markExplored(); // We're exploring this one.

			// Now process nodes in the queue on each iteration:
			while (!queue.isEmpty()) {
				int front = queue.get(0).intValue(); // Get first index in Q
				queue.remove(0); // Pop the queue (remove the front item)

				if (front == endIdx) {
					// We're done. We found the end node!
					return true;
				}

				// Otherwise check neighbors and insert into the queue, and mark we visited them.
				// Look at the index of each adjacent node from this one.
				for (int nidx = 0; nidx < adj.get(front).size(); nidx++) {
					int adjacent = adj.get(front).get(nidx).intValue();
					if (!nodes.get(adjacent).isExplored()) {
						// We mark them explored as they go into the queue so we never add them to the queue again.
						// Otherwise the algorithm won't terminate... it will just run around the graph forever.
						nodes.get(adjacent).markExplored();
						// We're storing breadcrumbs in the nodes so we can recover the path when we're done.
						nodes.get(adjacent).setParent(front); // We got here from node in front of Q.

						// Distances are optional at the moment, but this adds distance thus far to adjacent distance.
						nodes.get(adjacent).setDistance(nodes.get(front).distance(nodes.get(adjacent))+ nodes.get(front).getDistance());
						// Put the adjacent node into the queue to explore later in the loop.
						queue.add(adjacent);
					} // End processing unexplored adjacent node.
				} // End loop over adjacent nodes.
			} // End queue ! empty.

			// If we get down here we never found our goal.
			return false;
		}

		// This function takes a graph that has a succesfully found path in it and the end node
		// It uses the parent links to recover the found path through the graph in order as nodes.
		public ArrayList<Node> recoverPath(int endIdx) {
			ArrayList<Node> path = new ArrayList<Node>();
			path.add(nodes.get(endIdx));

			while (path.get(0).getParent() >= 0) {
				// Add the parent of this node as the front of the path unless it has no parent.
				path.add(0, nodes.get(path.get(0).getParent()));
			}

			return path;
		}
	}

	public ArrayList<Node> getPath(int startIdx, int endIdx){
		graph.findPath(startIdx, endIdx);
		return graph.recoverPath(endIdx);
	}

	public ArrayList<Node> getPathFromClosest(double shoulder, double elbow, double wrist, int endIdx){
		return getPath(graph.findClosestNode(shoulder, elbow, wrist), endIdx);
	}

	

}