//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

#include <unordered_map>


namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	//class AStarPlanningState : public SteerLib::BestFirstSearchPlanner
	
	class AStarAction : public SteerLib::DefaultAction<int> {

	};

	class STEERLIB_API AStarPlanner 
		//: public SteerLib::BestFirstSearchPlanner<PlanningDomain, 
		//PlanningState, 
		//SteerLib:: PlanningAction = DefaultAction<PlanningState> >
	{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
			// A#4 Added
			// SearchNode definition
			struct SearchNode {
				int agent_index = 0;
				int parent_agent_index = 0;
				double f = 0;
				double g = 0;
				double h = 0;
				bool operator<(const SearchNode& rhs) const
				{
					if (f < rhs.f)
						return true;
					else if (f == rhs.f)
					{
						if (g < rhs.g)
							return true;
					}
					return false;
				}
				bool operator>(const SearchNode& rhs) const
				{
					if (f > rhs.f)
						return true;
					else if (f == rhs.f)
					{
						if (g > rhs.g)
							return true;
					}
					return false;
				}
			};

			typedef std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> searchnode_priority_queue;

			typedef std::unordered_map<int, int> iumap;
			void AStarPlanner::print_came_from(const iumap& came_from);

			bool AStarPlanner::reconstruct_path(SpatialDataBaseInterface * _gSpatialDatabase,
					std::vector<Util::Point>& agent_path, const std::unordered_map<int, int>& came_from, const int& start_cell_idx, const int& goal_cell_idx, searchnode_priority_queue& closedSet);

			bool AStarPlanner::neighbour_in_openset(SearchNode& neighbour_node, searchnode_priority_queue& openSet);
			bool AStarPlanner::neighbour_in_closedset(SearchNode& neighbour_node, searchnode_priority_queue& closedSet);
			void AStarPlanner::move_neighbor_from_closed_to_openSet(SearchNode& neighbour_node,
				searchnode_priority_queue& closedSet, searchnode_priority_queue& openSet);
			void AStarPlanner::print_openSet(searchnode_priority_queue& openSet, SpatialDataBaseInterface * _gSpatialDatabase);

			void AStarPlanner::prune_openSet_closed(searchnode_priority_queue& openSet, SearchNode current_closed_node);

			void AStarPlanner::prune_openSet(searchnode_priority_queue& openSet, SearchNode current_node);

			void AStarPlanner::print_closedSet(searchnode_priority_queue& closedSet, SpatialDataBaseInterface * _gSpatialDatabase);

			bool AStarPlanner::weightedAStar(SpatialDataBaseInterface * _gSpatialDatabase, std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal,
				double heuristics_type, double epsilon, AStarPlanner * planner);

			double AStarPlanner::calculate_h(Util::Point& _start, Util::Point& _goal,
				const double& type, SpatialDataBaseInterface * _gSpatialDatabase);

			double AStarPlanner::calculate_f(Util::Point& start, Util::Point& goal, double g_val, double& euristic_type, double& epsilon, SpatialDataBaseInterface * _gSpatialDatabase);
	};

}


#endif
