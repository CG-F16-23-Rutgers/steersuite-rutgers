//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"

#include <unordered_map>
#include "planning/BestFirstSearchPlanner.h"



#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
int offset;
std::vector<Util::Point> path_plan;

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}


	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}


	double AStarPlanner::calculate_h(Util::Point& _start, Util::Point& _goal,
		const double& type, SpatialDataBaseInterface * _gSpatialDatabase) {
		Util::Point start;
		Util::Point goal;
		int start_idx, goal_idx;
		/*
		std::cout <<
			" type: " << type
			<< std::endl;
		*/
		start_idx = _gSpatialDatabase->getCellIndexFromLocation(_start);
		_gSpatialDatabase->getLocationFromIndex(start_idx, start);
		goal_idx = _gSpatialDatabase->getCellIndexFromLocation(_goal);
		_gSpatialDatabase->getLocationFromIndex(goal_idx, goal);

		double dx, dz;
		dx = fabs(start.x - goal.x);
		dz = fabs(start.z - goal.z);

		if (type == 1)
			// do Euclidian calculation 
		{
			/*
			std::cout <<
				" euclidian: " << sqrt(dx*dx + dz*dz)
				<< std::endl;
			*/
			return sqrt(dx*dx + dz*dz);
		}
		//do Manhattan calculation
		else if (type == 2) {
			/*
			std::cout <<
				" manhattan: " << dx + dz
				<< std::endl;
			*/
			return dx + dz;

		}
		return -1; // this is an error
	}

	double AStarPlanner::calculate_f(Util::Point& start, Util::Point& goal, double g_val, double& euristic_type, double& epsilon, SpatialDataBaseInterface * _gSpatialDatabase) {
		double result;

		return result = g_val + (epsilon * calculate_h(start, goal, euristic_type, _gSpatialDatabase));

		//return -1; // this is an error
	}



	void AStarPlanner::print_came_from(const iumap& came_from) {
		// starting from the bottom of the came_from list 
		// push all Nodes Backwards
		for (auto iter = came_from.end(); iter != came_from.begin(); --iter) {
			// DBG
			//
			std::cout << "Printing came_from: "
				" | iter.first: " << iter->first <<
				" | iter.second: " << iter->second

				<< std::endl;
			//

		}

	}

	bool AStarPlanner::reconstruct_path(SpatialDataBaseInterface * _gSpatialDatabase,
		std::vector<Util::Point>& agent_path, const std::unordered_map<int, int>& came_from, const int& start_cell_idx, const int& goal_cell_idx, searchnode_priority_queue&  closedSet) {
		bool path_reconstructed;
		// starting from the bottom of the came_from list 
		// push all Nodes Backwards
		/*
		for (auto iter = came_from.end(); iter != came_from.begin(); --iter) {
			// DBG
			//
			std::cout << "reconstruct_path - Checking hash content: "
				" | iter.first.index: " << iter->first <<
				" | iter.second.capacity: " << iter->second
				<< std::endl;
			//

			int agent_index = iter->first;
			Util::Point nodeLocation;
			_gSpatialDatabase->getLocationFromIndex(agent_index, nodeLocation);
			std::cout << "nodeLocation: " << nodeLocation  << std::endl;
			agent_path.push_back(nodeLocation);
		}
		*/
		// print agent_path upon entering reconstruct_path
		/*
		std::cout << "print agent_path upon entering reconstruct_path" << std::endl;
		for (int i = 0; i < agent_path.size(); ++i) {
			std::cout << "agent_path[" << i << "]" << agent_path[i]
				<< std::endl;
		}
		std::cout << "print start/goal upon entering reconstruct_path:" << std::endl;
		*/

		// goal location
		Util::Point goal_location;
		Util::Point start_location;
		_gSpatialDatabase->getLocationFromIndex(goal_cell_idx, goal_location);
		//std::cout << "Goal location: " << goal_location << std::endl;
		//std::cout << "Goal location index: " << goal_cell_idx << std::endl;
		_gSpatialDatabase->getLocationFromIndex(start_cell_idx, start_location);
		//std::cout << "Start location: " << start_location << std::endl;
		//std::cout << "Start location index: " << start_cell_idx << std::endl;



		std::unordered_map<int, int> hash;
		std::vector<int> child_loc;
		std::vector<int> parent_loc;

		// loading closedSet points to child_loc and parent_loc vectors
		SearchNode t_node;
		while (!closedSet.empty()) {
			t_node = closedSet.top();
			// DBG
			// output while processing
			// std::cout << "closedSet: " << t_node.agent_index << " | " << t_node.parent_agent_index << std::endl;
			child_loc.push_back(t_node.agent_index);
			parent_loc.push_back(t_node.parent_agent_index);
			closedSet.pop();
		}

		// DBG
		// print all vectors items
		/*
		std::cout << "print all vectors items: " << std::endl;
		for (int i = 0; i < child_loc.size(); ++i) {
			std::cout << "child | parent loc: " << child_loc[i] << " | " << parent_loc[i] << std::endl;
		}
		*/
		// rebuild from GOAL backwards

		// we have the goal index - goal_cell_idx

		int vector_seek_idx = goal_cell_idx;
		bool root_reached = false;
		Util::Point location;
		std::vector<Util::Point> tmp_path;

		int i = 0;
		// do just goal first
		//std::cout << "pushing agent path in reverse order: " << std::endl;
		while (child_loc[i] != parent_loc[i] || (!(root_reached))) {
			for (i = 0; i < child_loc.size(); ++i) {
				// DBG
				// output while processing
				/*
				std::cout
					<< " | child_loc[i]: " << child_loc[i]
					<< " | parent_loc[i]: " << parent_loc[i]
					<< " | vector_seek_idx: " << vector_seek_idx
					<< std::endl;
				*/
				if (child_loc[i] == vector_seek_idx) {
					// DBG
					// std::cout << "vector_seek_idx found: " << vector_seek_idx << std::endl;
					_gSpatialDatabase->getLocationFromIndex(child_loc[i], location);
					tmp_path.push_back(location);					
					vector_seek_idx = parent_loc[i];
					if (child_loc[i] == parent_loc[i])
						root_reached = true;
					break;

				}
			}
		};

		// print tmp_path
		/*
		std::cout << "print tmp_path upon leaving reconstruct_path" << std::endl;
		for (int i = 0; i < tmp_path.size(); ++i) {
			std::cout << "tmp_path: " << tmp_path[i] << std::endl;
		}
		*/

		//reverse waypoints
		for (i = tmp_path.size()-1; i >= 0; --i) {
			agent_path.push_back(tmp_path[i]);

		}


		/*

		// push all vectors to agent_path
		for (int i = 0; i < child_loc.size(); ++i) {

			if (child_loc[i] != parent_loc[i]) { // first item

				_gSpatialDatabase->getLocationFromIndex(child_loc[i], location);
				std::cout << "location: " << location << std::endl;
				agent_path.push_back(location);
			}
		}
		*/
		path_reconstructed = true;

		// DBG
		// print agent_path
		/*
		std::cout << "print agent_path upon leaving reconstruct_path" << std::endl;
		for (int i = 0; i < agent_path.size(); ++i) {
			std::cout << "agent_path: " << agent_path[i] << std::endl;
		}
		*/


		/*



		// get the root id
		// push the root to agent_path
		agent_path.clear();
		int root_id;
		for (auto iter = hash.begin(); iter != hash.end(); ++iter) {
			if (iter->first == iter->second)
				root_id = iter->first;

			Util::Point node_location;
			_gSpatialDatabase->getLocationFromIndex(root_id, node_location);
			//reconstructed_path.push_back(node_location);
			//path_plan.push_back(node_location);
		}

		offset = reconstructed_path.size() - 1;
		std::cout << "offset: " << offset << std::endl;


		*/

		// print all reconstructed_path items
		/*
		int i = 0;
		for (auto iter = reconstructed_path.begin(); iter != reconstructed_path.end(); ++iter, ++i) {
			std::cout << "agent_path: " << reconstructed_path[i] << std::endl;
		}
		*/

		// print all path_plan items
		/*
		int i = 0;
		for (auto iter = path_plan.begin(); iter != path_plan.end(); ++iter, ++i) {
		std::cout << "path_plan with iterator: " << path_plan[i] << std::endl;
		}
		*/




		/*
		// locate the root item
		int root_idx;
		for (auto iter = hash.begin(); iter != hash.end(); ++iter) {
			std::cout << "hash: " << iter->first << " | " << iter->second << std::endl;
		}

		std::cout << "root_id: " << root_id << std::endl;

		// print agent_path after pushing root
		std::cout << "print agent_path after pushing root" << std::endl;
		std::cout << "size: " << agent_path.size() << std::endl;
		for (int i = 0; i < agent_path.size(); ++i) {
			std::cout << "agent_path[" << i << "]" << agent_path[i]
				<< std::endl;
		}
		//agent_path.erase(agent_path.begin());
		//agent_path.erase(agent_path.begin()+1);
		//agent_path.erase(agent_path.begin()+2);
		std::cout << "agent path size: " << agent_path.size() << std::endl;


		// starting with root_id
		// push the rest and remove each added entry
		int last_value;
		bool reached_root = false;
		while (!(reached_root)) {
			std::cout << "hash size: " << hash.size() << std::endl;
			for (auto iter = hash.begin(); iter != hash.end(); ++iter) {
				if (iter->first == iter->second)
					reached_root = true;
				if (iter->second == root_id) {
					std::cout << "root id: " << root_id << std::endl;
					root_id = iter->first;
					Util::Point node_location;
					_gSpatialDatabase->getLocationFromIndex(root_id, node_location);
					agent_path.push_back(node_location);
					last_value = iter->second;
					std::cout << "new root id: " << root_id
						<< " | last_value: " << last_value
						<< std::endl;
					continue;
				}
			}

			// delete the entry from hash
			//hash.erase(last_value);
			//if (hash.size() == 2)
			//	break;
		}

		// print agent_path
		for (int i = 0; i < agent_path.size(); ++i) {
			std::cout << "agent_path[" << i << "]" << agent_path[i]
				<< std::endl;
		}
		*/
		return path_reconstructed;
	}

	bool AStarPlanner::neighbour_in_openset(SearchNode& neighbour_node, searchnode_priority_queue& openSet) {
		bool in_openSet = false;
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;

		while (!openSet.empty() && !in_openSet) {
			SearchNode node = openSet.top();
			openSet.pop();
			temp_queue.push(node);

			if (node.agent_index == neighbour_node.agent_index) {
				in_openSet = true;
				break;
			}
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			openSet.push(node);
		}

		return in_openSet;
	}

	bool AStarPlanner::neighbour_in_closedset(SearchNode& neighbour_node, searchnode_priority_queue& closedSet) {
		bool in_closedSet = false;
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;

		while (!closedSet.empty() && !in_closedSet) {
			SearchNode node = closedSet.top();
			closedSet.pop();
			temp_queue.push(node);
			/*
			std::cout
				<< "checking node: "
				<< " | node.agent_index: " << node.agent_index
				<< " | neighbour_node.agent_index: " << neighbour_node.agent_index
				<< std::endl;
			*/
			if (node.agent_index == neighbour_node.agent_index)
			{
			/*
				std::cout 
					<< "match found: " << node.agent_index 
					<< " | " << node.agent_index
					<< std::endl;
					*/
				in_closedSet = true;
				break;

			}
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			closedSet.push(node);
		}

		return in_closedSet;
	}

	void AStarPlanner::move_neighbor_from_closed_to_openSet(SearchNode& neighbour_node,
		searchnode_priority_queue& closedSet, searchnode_priority_queue& openSet) {
		bool node_found = false;
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;

		while (!closedSet.empty() && !node_found) {
			SearchNode node = closedSet.top();
			closedSet.pop();
			temp_queue.push(node);

			if (node.agent_index == neighbour_node.agent_index) {

				node_found = true;
				openSet.push(node); // this is the only push to open
			}
			break;
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			closedSet.push(node);
		}

		return;
	}

	void AStarPlanner::prune_openSet_closed(searchnode_priority_queue& openSet, SearchNode current_closed_node) {

		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;

		std::cout << "prunning openSet from nodes in CloseSet start: "
			<< " id: " << current_closed_node.agent_index
			<< " | size: " << openSet.size()
			<< std::endl;

		while (!openSet.empty()) {
			SearchNode node = openSet.top();
			openSet.pop();

			// if id are different - push to the temp queue 
			std::cout << "looping the openSet: "
				<< " node.agent_index: " << node.agent_index
				<< " | current_closed_node.agent_index: " << current_closed_node.agent_index
				<< std::endl;
			if (node.agent_index != current_closed_node.agent_index) {
				std::cout << "adding this node: " << std::endl;
					temp_queue.push(node);
			}
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			openSet.push(node);
		}
		return;
	}

	void AStarPlanner::prune_openSet(searchnode_priority_queue& openSet, SearchNode current_node) {

		//output the OPEN set 
		// DBG
		// start openSet print
		/*
		std::cout << "prunning openSet start: " 
			<< " id: " << current_node.agent_index
			<< " | size: " << openSet.size()
			<< std::endl;
		*/
		/*
		std::cout << "closedSet size:" << closedSet.size()
		<< std::endl;
		*/
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;
		while (!openSet.empty()) {
			SearchNode node = openSet.top();
			openSet.pop();
			// here is the prunning
			/*
			std::cout << "openSet size: " << node.agent_index
				<< " | current_node.agent_index: " << current_node.agent_index
				<< std::endl;
			*/

			// if id are different - push to the temp queue 
			if (node.agent_index != current_node.agent_index)
				temp_queue.push(node);
			// if id are the same and node in openSet has g > from recently added node 
			// keep new one - toss the old
			else if (node.agent_index == current_node.agent_index && node.f >= current_node.f)
				temp_queue.push(node);
				//temp_queue.push(current_node);
			// if id are the same and node in openSet has g < from recently added node 
			// keep old one - toss the new one
			//else 
			//	temp_queue.push(node);
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			openSet.push(node);
		}
		// end prunning print
		/*
		std::cout << "prunning openSet end: "
			<< " | size: " << openSet.size()
			<< std::endl;
		*/
		return;
	}


	void AStarPlanner::print_openSet(searchnode_priority_queue& openSet, SpatialDataBaseInterface * _gSpatialDatabase) {

		//output the OPEN set 
		// DBG
		// start openSet print
		/*
		std::cout << "openSet size: " << openSet.size()
			<< std::endl;
		std::cout << "closedSet size:" << closedSet.size()
			<< std::endl;
		*/
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;
		while (!openSet.empty()) {
			SearchNode node = openSet.top();
			openSet.pop();
			temp_queue.push(node);
			Util::Point location;
			_gSpatialDatabase->getLocationFromIndex(node.agent_index, location);
			// DBG
			// output of the openSet
			/*
			std::cout << "Printing openSet:"
				<< " | index: " << node.agent_index
				<< " | parent index: " << node.parent_agent_index
				<< " | f: " << node.f
				<< " | g: " << node.g
				<< " | h: " << node.h
				<< " | loc.: " << location
				<< std::endl;
			*/
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			openSet.push(node);
		}
		// end openSet print

		return;
	}

	void AStarPlanner::print_closedSet(searchnode_priority_queue& closedSet, SpatialDataBaseInterface * _gSpatialDatabase) {

		//output the OPEN set 
		// DBG
		// start openSet print
		/*
		std::cout << "closedSet size: " << closedSet.size()
			<< std::endl;
		*/
		/*
		std::cout << "closedSet size:" << closedSet.size()
		<< std::endl;
		*/
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> temp_queue;
		while (!closedSet.empty()) {
			SearchNode node = closedSet.top();
			closedSet.pop();
			temp_queue.push(node);
			Util::Point location;
			_gSpatialDatabase->getLocationFromIndex(node.agent_index, location);
			// DBG 
			// debug print of the closedSet
			/*
			std::cout << "Printing closedSet:"
				<< " | index: " << node.agent_index
				<< " | parent_index: " << node.parent_agent_index
				<< " | f: " << node.f
				<< " | g: " << node.g
				<< " | h: " << node.h
				<< " | loc.: " << location
				<< std::endl;
			*/
		}

		// push back all popped nodes to the queue
		while (!temp_queue.empty()) {
			SearchNode node = temp_queue.top();
			temp_queue.pop();
			closedSet.push(node);
		}
		// end openSet print

		return;
	}




	bool AStarPlanner::weightedAStar(SpatialDataBaseInterface * _gSpatialDatabase, std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal,
		double heuristics_type, double epsilon, AStarPlanner * planner) {

		//int offset;
		// declaration and init
		bool plan_generated;
		int start_cell_index, goal_cell_index;
		//std::unordered_map<int, int> come_from;
		std::vector<Util::Point> reconstructed_path;


		//calculate indices of start and goal points
		start_cell_index = _gSpatialDatabase->getCellIndexFromLocation(start);
		goal_cell_index = _gSpatialDatabase->getCellIndexFromLocation(goal);
		/*
		std::cout <<
			" | start_ind: " << start_cell_index <<
			" | goal_ind: " << goal_cell_index
			<< std::endl;
			*/
		// closedset - set of nodes already evaluated
		// openset  - set of tentative nodes to be evaluated
		// came_from - map of navigated nodes

		// do closedset as priority queue - it is a queue of points
		// location of the 

		// Define OpenSet, ClosedSet and CameFrom
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openSet;
		std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> closedSet;
		//std::set<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> closedSet;
		//std::unordered_map<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> came_from;
		std::unordered_map<int, int> came_from;
		//init hash map
		came_from.clear();
		came_from.size();
		//came_from.insert(-1) = -1;

		// Build and insert the starting cell to the OpenSet - line 1
		SearchNode currentNode;
		currentNode.agent_index = start_cell_index;
		currentNode.parent_agent_index = start_cell_index;
		currentNode.g = 0;
		currentNode.h = calculate_h(start, goal, heuristics_type, _gSpatialDatabase);
		currentNode.f = calculate_f(start, goal, currentNode.g, heuristics_type, epsilon, _gSpatialDatabase);
		openSet.push(currentNode);
		// DBG2
		//print_openSet(openSet, _gSpatialDatabase);


		// while openSet is not empty - line 2
		while (!openSet.empty()) {
			// load top of the OpenSet and move it to the closedSet
			// DBG
			// std::cout << "*** line 1 - restarting the while loop; openSet size: " << openSet.size()
			//	<< std::endl;
			// DBG2
			// printing open set for each openset element
			Util::Point curr_node_location;
			_gSpatialDatabase->getLocationFromIndex(currentNode.agent_index, curr_node_location);
			/*
			std::cout << "*** line 1 - current node index: " 
				<< currentNode.agent_index << " | location: " << curr_node_location
				<< std::endl;
			std::cout << "*** line 1 - printing openSet size: " << openSet.size() << std::endl;
			print_openSet(openSet, _gSpatialDatabase);
			std::cout << "*** line 1 - printing closedSet - size: " << openSet.size() << std::endl;
			print_closedSet(closedSet, _gSpatialDatabase);
			*/
			currentNode = openSet.top();
			//
			Util::Point curr_loc;
			_gSpatialDatabase->getLocationFromIndex(currentNode.agent_index, curr_loc);
			//DBG
			// debug - start processing of new currentNode
			/*
			std::cout << "****** line 12 - new currentNode: "
				<< " | currentNode.id: " << currentNode.agent_index
				<< " | currentNode.location: " << curr_loc
				<< std::endl;
			*/

			// add currentNode to CLOSED 
			// if current node 
			if (!(neighbour_in_closedset(currentNode, closedSet))) {
				closedSet.push(currentNode);
				// DBG3
				// prune open set from any nodes already in closedSet
				print_openSet(openSet, _gSpatialDatabase);
				print_closedSet(closedSet, _gSpatialDatabase);
				// DBG3
				//if (openSet.size() > 1) 
				//	prune_openSet_closed(openSet, currentNode);
				//came_from[currentNode.agent_index] = currentNode.parent_agent_index;
			}
			else
				// skip this node - it is a duplicate
				continue;
			// DBG
			// debug print of closedSet
			// print_closedSet(closedSet, gSpatialDatabase);
			
			// remove top of OPEN
			openSet.pop();
			// DBG
			//print_openSet(openSet, gSpatialDatabase);
			/*
			std::cout << "*** line 12 - openSet size:" << openSet.size()
				<< std::endl;
			*/

			// if current cell is different then goal cell - line 5
			// *** line 13
			// goal has been reached

			if (currentNode.agent_index == goal_cell_index) {
				/*
				std::cout << "*** line 13 - openSet size:" << openSet.size() 
					<< " | closedSet.size(): "<< closedSet.size()
					<< std::endl;
				*/

				// copy the come_from into agent_path for computePath use
				// in reversed order
				plan_generated = reconstruct_path(_gSpatialDatabase, agent_path, came_from, start_cell_index, goal_cell_index, closedSet);
				plan_generated = true;

				// DBG
				// print agent_path
				/*
				std::cout << "print agent_path from weightedAStar" << std::endl;
				for (int i = 0; i < agent_path.size(); ++i) {
					std::cout << "agent_path: " << agent_path[i] << std::endl;
				}
				*/


				// path other parameters: length, cost ?!
				break;
			}

			// get the neighbours of current cell - line 6
			// using 8-connected neighbours 
			// going counterclockwise starting from 3 o'clock
			int dx[] = { 1,1,0,-1,-1,-1,0,1 };
			int dz[] = { 0,1,1,1,0,-1,-1,-1 };
			unsigned int currNode_ind_x, currNode_ind_z;
			//std::vector<int> neighbours_list; //vector of CellIndices - not coordinates!
			std::vector<SearchNode> neighbour_nodes; //vector of SearchNodes

			//
			_gSpatialDatabase->getGridCoordinatesFromIndex(currentNode.agent_index, currNode_ind_x, currNode_ind_z);
			Util::Point n_location;
			_gSpatialDatabase->getLocationFromIndex(currentNode.agent_index, n_location);
			/*
			std::cout << "***line6 - Current node:"
				<< " | currNode_x: " << currNode_ind_x
				<< " | currNode_z: " << currNode_ind_z
				<< " | [x,z]: " << n_location
				<< std::endl;
			*/

			for (int i = 0; i < 8; i++) {
				int x_next = currNode_ind_x + dx[i];
				int z_next = currNode_ind_z + dz[i];


				/*
				std::cout << "***line6 - Generating neighbours:"
					<< " | i: " << i
					<< " | currNode_x: " << currNode_ind_x
					<< " | currNode_z: " << currNode_ind_z
					<< " |  dx[i]: " << dx[i]
					<< " |  dz[i]: " << dz[i]
					<< " | currNode_x: " << x_next
					<< " | currNode_z: " << z_next
					<< std::endl;
				*/
				if (x_next >= 0 && x_next < _gSpatialDatabase->getNumCellsX() &&
					z_next >= 0 && z_next < _gSpatialDatabase->getNumCellsZ()) {
					/*
					std::cout << "Coordinates:"
						<< " | xnext: " << x_next
						<< " | znext: " << z_next
						<< std::endl;
					*/
					int newNeighbour_index = _gSpatialDatabase->getCellIndexFromGridCoords(x_next, z_next);
					Util::Point location; location.x = x_next; location.z = z_next;
					int index = _gSpatialDatabase->getCellIndexFromLocation(location);
					SearchNode new_neighbour_node;
					new_neighbour_node.agent_index = newNeighbour_index ;
					// generate new neigbor only if it is not already in closed set

					if (!(neighbour_in_closedset(new_neighbour_node, closedSet))) {
						/*
						std::cout << "Generating neighbour:"
							<< " | index: " << newNeighbour_index
							<< std::endl;
							*/
						neighbour_nodes.push_back(new_neighbour_node);
					}
				}

			} // end for for generating neighbours


			// iterate through the neighbours list  - for each neighbours_list[i] of currNode - line 7
			// *** line 16
			SearchNode min_g_neighbour;
			min_g_neighbour.g = INFINITY;
			for (int i = 0; i < neighbour_nodes.size(); i++) {

				// DBG
				/*
				std::cout << "*** line 16 - iterate neighbours:"
					<< " | i: " << i
					<< " | neighbour_nodes[i].index: " << neighbour_nodes[i].agent_index
					<< std::endl;
				//fprint_openSet(openSet, gSpatialDatabase);
				print_closedSet(closedSet, gSpatialDatabase);
				*/
				// if neighbor in closedset
				// if cell is accesible - continue

				// end line 8

				/*
				Util::Point n_location;
				_gSpatialDatabase->getLocationFromIndex(neighbours_list[i], n_location);
				*/
				/*
				std::cout << "***line 8 - Checking neighbour in closedset:"
					//<< " | neighbours_list.size(): " << neighbours_list.size()
					//<< " | currentNode index: " << neighbours_list[i]
					//<< " | hasanyitems: " << _gSpatialDatabase->hasAnyItems(neighbours_list[i])
					<< " | [x,z]: " << n_location
					//<< " | canbetraverved: " << planner->canBeTraversed(neighbours_list[i])
					<< std::endl;
				*/
				// if neighbor can be reached  
				//_gSpatialDatabase->hasAnyItems(neighbours_list[i]) &&

				// if cell is reachabele and neighbour is in openSet - line 9
				/*
				std::cout << "***16-Current neighbour is in openSet line 9:"
					<< " | i: " << i
					<< " | neighbour_nodes[i].index: " << neighbour_nodes[i].agent_index
					<< std::endl;
				*/

				// traversable
				// planner->canBeTraversed(neighbour_nodes[i].agent_index)
				// *** line 161
				// if neighbour is not in closed set 
				if (!(neighbour_in_closedset(neighbour_nodes[i], closedSet))) {
					// DBG
					/*
					std::cout << "*** line 161 - Current neighbour is not in the closedSet line 9:"
						<< " | i: " << i
						<< " | neighbour_nodes[i].index: " << neighbour_nodes[i].agent_index
						<< std::endl;
					*/

					// generate tentative_gValue - line 8
					Util::Point current_cell_location;
					gSpatialDatabase->getLocationFromIndex(currentNode.agent_index, current_cell_location);
					Util::Point	next_cell_location;
					gSpatialDatabase->getLocationFromIndex(neighbour_nodes[i].agent_index, next_cell_location);
					double cost_between;
					if ((fabs(current_cell_location.x - next_cell_location.x) +
						fabs(current_cell_location.z - next_cell_location.z)) == 1)
						cost_between = 1;
					else
						cost_between = sqrt(2);
					/*
					std::cout << "***line 8: Finding location: "
					<< " | current_cell_location: " << current_cell_location
					<< " | next_cell_location: " << next_cell_location
					<< " | v: " << cost_between
					<< std::endl;
					*/

					// DBG
					// *** line 162
					double tentative_gValue = currentNode.g + cost_between;
					/*
					std::cout << "*** after line 162: "
						<< " | i: " << i
						<< " | node index: " << currentNode.agent_index
						<< " | currentNode.g: " << currentNode.g
						<< " | tentative_gValue: " << tentative_gValue
						<< std::endl;
					*/


					// DBG
					// *** line 163
					/*
					std::cout << "*** before line 163"
						<< " | tentative_gValue: " << tentative_gValue
						<< " | neighbour_nodes[i].g: " << neighbour_nodes[i].g
						<< std::endl;
					*/
					/*
					std::cout << "*** line 1631 - neighbours values before recalc:"
						<< " | tentative_gValue: " << tentative_gValue
						<< " | neighbour_nodes[i].index: " << neighbour_nodes[i].agent_index
						<< " | f: " << neighbour_nodes[i].f
						<< " | g: " << neighbour_nodes[i].g
						<< " | h: " << neighbour_nodes[i].h
						<< std::endl;
					*/


					// *** line 1632
					double f_next, g_next, h_next;
					g_next = tentative_gValue;
					Util::Point h_point;
					_gSpatialDatabase->getLocationFromIndex(neighbour_nodes[i].agent_index, h_point);
					h_next = calculate_h(h_point, goal, heuristics_type, _gSpatialDatabase);
					f_next = calculate_f(h_point, goal, g_next, heuristics_type, epsilon, _gSpatialDatabase);
					neighbour_nodes[i].parent_agent_index = currentNode.agent_index;
					neighbour_nodes[i].f = f_next;
					neighbour_nodes[i].g = g_next;
					neighbour_nodes[i].h = h_next;


					// DBG
					/*
					std::cout << "*** line 1633 - pass 2 - neighbours values afer recalc: "
						<< " | tentative_gValue " << tentative_gValue
						<< " | neighbour_nodes[i].index: " << neighbour_nodes[i].agent_index
						<< " | f: " << neighbour_nodes[i].f
						<< " | g: " << neighbour_nodes[i].g
						<< " | h: " << neighbour_nodes[i].h
						<< std::endl;
					/*


					// show open set before pushing
					/*
					std::cout << "*** line 1634 - before pushing to openSet:" << openSet.size()
						<< std::endl;
					print_openSet(openSet, gSpatialDatabase);
					*/

					//set parent - add to came_from - line 19
					// *** line 1631
					/*
					std::cout << "*** line 1631 - before adding an item to came_from"
					<< 	" | neighbour_nodes[i].agent_index: " << neighbour_nodes[i].agent_index
						<< " | currentNode.agent_index: " << currentNode.agent_index

						<< std::endl;
					*/




					// if neighbour is not in the openSet and is traversible - add it to openSet
					// *** line 163 - node is traversable and is not in the open set
					/*
					std::cout << "*** line 163 - before - openSet size:" << openSet.size()
						<< " | closedSet.size(): " << closedSet.size()
						<< std::endl;
						*/
					if ((!(neighbour_in_openset(neighbour_nodes[i], openSet))) &&
						(planner->canBeTraversed(neighbour_nodes[i].agent_index))) {
						openSet.push(neighbour_nodes[i]);
						// openSet prunning
						prune_openSet(openSet, neighbour_nodes[i]);

						/*
						std::cout << "*** line 163 - after - openSet size:" << openSet.size()
							<< " | closedSet.size(): " << closedSet.size()
							<< std::endl;
							*/

						// show updated Open set
						// DBG
						/*
						std::cout << "*** line 163 - after pushing to openSet: " << openSet.size()
							<< std::endl;
						print_openSet(openSet, gSpatialDatabase);
						*/
					}
					else {
						if (neighbour_in_openset(neighbour_nodes[i], openSet) &&
							(planner->canBeTraversed(neighbour_nodes[i].agent_index))
							) {
							// DBG
							// std::cout << "*** line 164 - neighbor is in openSet: " << std::endl;
							if (neighbour_nodes[i].g < min_g_neighbour.g) {
								// DBG
								/*
								std::cout << "****** line 165 - neighbour_nodes[i].g < min_g_neighbour.g: "
									<< " | neighbour_nodes[i].g: " << neighbour_nodes[i].g
									<< " | min_g_neighbour.g: " << min_g_neighbour.g
									<< std::endl;
								
								// need to update existing  
								//came_from.erase();
								min_g_neighbour = neighbour_nodes[i];

								// update came_form neigbour
								//
								Util::Point loc;
								_gSpatialDatabase->getLocationFromIndex(min_g_neighbour.agent_index, loc);
								//came_from[min_g_neighbour.agent_index] = currentNode.agent_index;
								// ******
								//came_from.insert_or_assign(currentNode.agent_index,
								//	neighbour_nodes[i].agent_index);
								std::cout << "*** line 1631 - Just added an item to came_from" << std::endl;
								// DBG
								// print_came_from(came_from);
								*/
							}
						}
					}

					// DBG
					/*
					std::cout << "*** line 163 - Current neighbour is not in openSet or tentative_gValue <= neighbour_nodes[i].g: \n"
						<< " | i: " << i
						<< " | neighbour_nodes[i].index: " << neighbour_nodes[i].agent_index
						<< std::endl;
					*/
				}
			} //end neighbours for loop
			// do came_from update only after the node is finally selected from all neighbours 
			// we should have only one min_g_neighbour per current
		} //end while (!openSet.empty)

		return plan_generated;
	} //end routine


	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		bool path_appended;
		std::vector<Util::Point> plan_path;

		// TODO 
		/*
		std::cout << "\nIn A*"
			" | start: " << start <<
			" | goal: " << goal
			<< std::endl;
			*/

		// print agent_path
		/*
		std::cout << "print agent_path from computePath" << std::endl;
		for (int i = 0; i < agent_path.size(); ++i) {
			std::cout << "agent_path: " << agent_path[i] << std::endl;
		}
		*/



		// Calling Weighted A Star
		// heuristics type = 1 for euclidian 
		//					= 2 for manhattan
		double heuristics_type = 1;
		// epsilon is the heuristics weight
		double epsilon = 1;
		//
		if (agent_path.size() == 0) 
			path_appended = weightedAStar(_gSpatialDatabase, agent_path, start, goal, heuristics_type, epsilon, this);

		/*
		if (!append_to_path) {
			agent_path.clear();
			path_appended = weightedAStar(_gSpatialDatabase, agent_path, start, goal, heuristics_type, epsilon, this);
		}
		*/


		return path_appended;
	}
}