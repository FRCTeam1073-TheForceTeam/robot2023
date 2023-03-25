// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

/** Add your docs here. */
public class PathPlanner {
	Arm arm;
	public ArrayList<Node> nodes = new ArrayList<Node>();
	public final Node STOW = new Node(-3.87, 2.9, -1.21);
	public final Node STOW_INTERMEDIATE = new Node(-2.6, 2.8, -1.2);
	public final Node CUBE_TOP = new Node(-1.483, 3.570, -0.337);
	public final Node CUBE_MID = new Node(-1.72, 3.8, -0.26);
	public final Node CUBE_TOP_INTERMEDIATE = new Node(-2.0, 3.19, -0.77);
	public final Node CONE_TOP = new Node(-1.433, 3.143, 1.07);
	public final Node CONE_MID = new Node(-1.91, 3.451, 1.132);
	public final Node CONE_TOP_INTERMEDIATE = new Node(-2.0, 3.0, 0.0);

	public final Connection CONNECTION1 = new Connection(STOW, STOW_INTERMEDIATE);


	public class Node{
		double shoulder;
		double elbow;
		double wrist;

		public Node(double shoulder, double elbow, double wrist){
			this.shoulder = shoulder;
			this.elbow = elbow;
			this.wrist = wrist;
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