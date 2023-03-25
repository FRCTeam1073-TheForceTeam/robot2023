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

	public final Connection CONNECTION1 = new Connection(nodes.get(STOW), nodes.get(STOW_INTERMEDIATE));

	private Graph graph;

	public PathPlanner(){
		graph.nodes = nodes;
	}

	public class Node{
		public double shoulder;
		public double elbow;
		public double wrist;
		public boolean explored;
		//Arm.CartesianPosition position;
		//Arm.JointPositions angles;

		public Node(double shoulder, double elbow, double wrist){
			this.shoulder = shoulder;
			this.elbow = elbow;
			this.wrist = wrist;
			//angles = arm.new JointPositions(shoulder,elbow,wrist);
		}
	}

	public class Graph{
		public ArrayList<Node> nodes;
		public ArrayList<ArrayList<Integer>> adj;

		public Graph(){
			nodes = new ArrayList<Node>();
			adj = new ArrayList<ArrayList<Integer>>();
		}

		public void addNode(Node node){
			node.explored = false;
			nodes.add(node);
			adj.add(new ArrayList<Integer>());
		}

		public void addEdge(int a, int b){
			adj.get(a).add(b);
			adj.get(b).add(a);
		}

		public int findClosestNode(double shoulder, double elbow, double wrist){
			double bestDistnce = Double.MAX_VALUE;
			int bestIdx = 0;
			for(int i = 0; i < nodes.size(); i++){
				double distance = Math.pow((shoulder - nodes.get(i).shoulder), 2) + Math.pow((elbow - nodes.get(i).elbow), 2) + Math.pow((wrist - nodes.get(i).wrist), 2);
				if(distance < bestDistnce){
					bestDistnce = distance;
					bestIdx = i;
				}
			}
			return bestIdx;
		}

		public ArrayList<Node> findPath(int startIdx, int endIdx){
			ArrayList<Node> result = new ArrayList<Node>();

			return result;
		}
	}

	public class Connection{
		Node node1;
		Node node2;

		public Connection(Node node1, Node node2){
			this.node1 = node1;
			this.node2 = node2;
		}
	}

}