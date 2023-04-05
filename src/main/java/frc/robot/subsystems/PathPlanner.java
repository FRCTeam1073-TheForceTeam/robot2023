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
		// All ends
		graph.addNode(new Node(-3.87, 2.9, -1.21)); //STOW 0
		graph.addNode(new Node(-1.483, 3.570, -0.337)); //CUBE_TOP 1
		graph.addNode(new Node(-1.72, 3.8, -0.26)); //CUBE_MID 2
		graph.addNode(new Node(-1.129, 3.029, 1.399)); //CONE_TOP 3
		graph.addNode(new Node(-1.72, 3.545, 1.189)); //CONE_MID 4
		graph.addNode(new Node(-2.45, 3.47, -0.33)); //DOUBLE_SUB_CUBE 5
		graph.addNode(new Node(-2.111, 3.065, 1.484)); //DOUBLE_SUB_CONE 6
		graph.addNode(new Node(-1.05, 5.38, -1.21)); //GROUND_CUBE 7
		graph.addNode(new Node(-1.439, 4.765, 0.573)); //GROUND_CONE 8
		graph.addNode(new Node(-2.0719, 4.7582, 0.8069)); //STOW_ALT 9
		graph.addNode(new Node(-3.85, 2.89, 0.25)); //SINGLESUB_CUBE 10
		graph.addNode(new Node(-2.21, 4.549, -1.209)); // SINGLESUB_CONE 11

		// All intermediates
		graph.addNode(new Node(-3.23, 3.375, -1.2)); //STOW_INTERMEDIATE 12
		// GROUND_INTERMEDIATE 13
		// SINGLE_SUB_INTERMEDIATES 14
		// TOP_INTERMEDIATE 15
		// MID_INTERMEDIATE 16
		// DOUBLE_SUB_INTERMEDIATE 17

		//Main connections (connecting stow intermediate to all ends)
		graph.addEdge(12, 0);
		graph.addEdge(12, 1);
		graph.addEdge(12, 2);
		graph.addEdge(12, 3);
		graph.addEdge(12, 4);
		graph.addEdge(12, 5);
		graph.addEdge(12, 6);
		graph.addEdge(12, 7);
		graph.addEdge(12, 8);
		graph.addEdge(12, 9);
		graph.addEdge(12, 10);
		graph.addEdge(12, 11);
		
		//Other Connections
		// graph.addEdge(13, 7);
		// graph.addEdge(13, 8);
		// graph.addEdge(14, 10);
		// graph.addEdge(14, 11);
		// graph.addEdge(15, 1);
		// graph.addEdge(15, 3);
		// graph.addEdge(16, 2);
		// graph.addEdge(16, 4);
		// graph.addEdge(17, 5);
		// graph.addEdge(17, 6);
		// graph.addEdge(13, 14);
		// graph.addEdge(9, 15);
		// graph.addEdge(9, 16);
		// graph.addEdge(9, 17);
		// graph.addEdge(17, 15);
		// graph.addEdge(17, 16);
		// graph.addEdge(16, 15);
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