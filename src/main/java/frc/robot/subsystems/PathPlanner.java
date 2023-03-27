// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Arrays;

/** Add your docs here. */
public class PathPlanner {
	Arm arm;
	public ArrayList<Node> nodes = new ArrayList<Node>(Arrays.asList(
		new Node(-3.87, 2.9, -1.21), //STOW
		new Node(-2.6, 2.8, -1.2), //STOW_INTERMEDIATE
		new Node(-1.483, 3.570, -0.337), //CUBE_TOP
		new Node(-1.72, 3.8, -0.26), //CUBE_MID
		new Node(-2.0, 3.19, -0.77), //CUBE_TOP_INTERMEDIATE
		new Node(-1.433, 3.143, 1.07), //CONE_TOP
		new Node(-1.91, 3.451, 1.132), //CONE_MID
		new Node(-2.0, 3.0, 0.0) //CONE_TOP_INTERMEDIATE
	));


	public final int STOW = 0;
	public final int STOW_INTERMEDIATE = 1;
	public final int CUBE_TOP = 2;
	public final int CUBE_MID = 3;
	public final int CUBE_TOP_INTERMEDIATE = 4;
	public final int CONE_TOP = 5;
	public final int CONE_MID = 6;
	public final int CONE_TOP_INTERMEDIATE = 7;

	private Graph graph;

	public PathPlanner(){
		graph.nodes = nodes;
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
			node.explored = false;
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


}