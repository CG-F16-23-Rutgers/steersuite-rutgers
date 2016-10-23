//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
// #include <math.h>

// #include "util/Geometry.h"

/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.

#undef min
#undef max

//Global Steering Coefficients
#define AGENT_MASS 1.0f
#define QUEUEING_COEF 1.6f
#define COLL_AVOID_COEF .75f
//
#define MAX_QUEUE_AHEAD 0.75f
#define AHEAD_RADIUS 1.25f
#define VELOCITY_SCALAR 0.25f

//Collision Avoidance 
#define MAX_SEE_AHEAD 3.0f




struct SourceTargetCouple {
	int SourceID;
	std::string SourceName;
	int TargetID;
	std::string TargetName;
	friend bool operator<(SourceTargetCouple const& a, SourceTargetCouple const& b)
	{
		return -1;
	}
};

typedef SourceTargetCouple* SourceTargetCouplePtr;
std::set<SourceTargetCouple> SeekCouple;
std::set<SourceTargetCouple> FleeCouple;

struct AgentColorEntry {
	int agentID;
	std::string sourceName;
	Util::Color sourceColor;
	friend bool operator<(AgentColorEntry const& a, AgentColorEntry const& b)
	{
		return -1;
	}
};

typedef AgentColorEntry* AgentColorEntryPtr;
std::set<AgentColorEntry> AgentColorTable;



using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

// A3 definitions 

SocialForcesAgent::SocialForcesAgent()
{
	_SocialForcesParams.sf_acceleration = sf_acceleration;
	_SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
	_SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
	_SocialForcesParams.sf_query_radius = sf_query_radius;
	_SocialForcesParams.sf_body_force = sf_body_force;
	_SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
	_SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
	_SocialForcesParams.sf_agent_b = sf_agent_b;
	_SocialForcesParams.sf_agent_a = sf_agent_a;
	_SocialForcesParams.sf_wall_b = sf_wall_b;
	_SocialForcesParams.sf_wall_a = sf_wall_a;
	_SocialForcesParams.sf_max_speed = sf_max_speed;

	_enabled = false;
}

SocialForcesAgent::~SocialForcesAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
	Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	// getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

SteerLib::EngineInterface * SocialForcesAgent::getSimulationEngine()
{
	return _gEngine;
}

void SocialForcesAgent::setParameters(Behaviour behave)
{
	this->_SocialForcesParams.setParameters(behave);
}

void SocialForcesAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled == true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	//DBG
	/*
	std::cout << std::endl;

	std::cout << "resetting agent " << id() << std::endl;
	*/
	_waypoints.clear();
	_midTermPath.clear();
	
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.5f, _position.z - _radius, _position.z + _radius);

	//insert in AgentColorTable
	AgentColorEntry currEntry;
	currEntry.agentID = id();
	currEntry.sourceName = initialConditions.name;
	currEntry.sourceColor = initialConditions.color;
	AgentColorTable.insert(currEntry);
	//end insert in AgentColorTable


	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
	*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;

	// std::cout << "inital colour of agent " << initialConditions.color << std::endl;
	if (initialConditions.colorSet == true)
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.5f, _position.z - _radius, _position.z + _radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		getSimulationEngine()->getSpatialDatabase()->addObject(dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		//DBG std::cout << "new position is " << _position << std::endl;
		//DBG std::cout << "new bounds are " << newBounds << std::endl;
		//DBG std::cout << "reset update " << this << std::endl;
		getSimulationEngine()->getSpatialDatabase()->updateObject(dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// 
		engineInfo->getSpatialDatabase()->updateObject(this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	bool skippedGoal = false;
	for (unsigned int i = 0; i < initialConditions.goals.size() && !skippedGoal; i++) {
		//printout current goal
		//DBG
		/*
		 std::cout << "ic.name: "<< initialConditions.name <<" | goal# " << i
			 << " | type: " << initialConditions.goals[i].goalType
			 << " | random?: " << initialConditions.goals[i].targetIsRandom
			 << std::endl;
		*/
		// if goal type is static seek target - add to the goal list
		if (initialConditions.goals[i].goalType == GOAL_TYPE_IDLE)
		{
			//DO NOTHING
			break;
		}

		else if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
			initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation =
					getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				_currentGoal.targetLocation = _goal.targetLocation;

			}
			else
			{
				//AgentGoalInfo _goal;

				_goalQueue.push(initialConditions.goals[i]);
				//DBG
				/*
				std::cout
					<< "ic.name: " << initialConditions.name
					<< " | ic.targetname " << initialConditions.goals[i].targetName
					<< std::endl;
					*/
			}
		}
		// if goal type is dynamic seek or flee target
		else if (initialConditions.goals[i].goalType == GOAL_TYPE_SEEK_DYNAMIC_TARGET)
		{
			/**/
			//show before inserting
			/**
			std::cout
				<< "Before insert - Listing initial condiitons per agent - goal id i: " << i 
				<< " | id(): " << id() // it->SourceName 
				<< " | source.name: " << initialConditions.name
				<< " | targetName: " << initialConditions.goals[i].targetName
				<< std::endl;
			*/
			_goalQueue.push(initialConditions.goals[i]);
			

			//insert in SeekCouple 
			SourceTargetCouple couplePtr;
			couplePtr.SourceID = id();
			couplePtr.SourceName = initialConditions.name;
			couplePtr.TargetName = initialConditions.goals[i].targetName;
				AgentInitialConditions aic;
				aic = getAgentConditions(this);
				couplePtr.TargetID = stoi((aic.name).substr(5));
			SeekCouple.insert(couplePtr);
			// end insert routine


			//get the id of the target agent 
			/* testing 
			SteerLib::AgentGoalInfo agi;
			SteerLib::AgentInitialConditions aic;
			agi.targetName;
			aic.name;
			AgentInterface ai;
			ai.agentNeighbors_
			*/
				//tmp_agent = dynamic_cast<SocialForcesAgent*> (*neighbour);
			/*
				std::cout
					<< "0000-getting the tagretagent: " << initialConditions.goals[i].targetName
					<< " | targetName: " << initialConditions.goals[i].targetName
					<< " | source.name: " << initialConditions.name
					<< " | : " 
					<< std::endl;
					*/
			
			/*
			std::cout
				<< "After insert: " << SeekCouple.size()
				<< std::endl;
			*/
			

			//DBG
			//show after insert
			/*
			std::set<SourceTargetCouple>::iterator it;
			for (it = SeekCouple.begin(); it != SeekCouple.end(); it++)
			{
				std::cout
					<< "After insert - it->SourceName: " << it->SourceName // it->SourceName 
					<< " | it->TargetName: " << it->TargetName
					<< std::endl;
			}
			*/

					
		}

		else if (initialConditions.goals[i].goalType == GOAL_TYPE_FLEE_DYNAMIC_TARGET)
		{

			_goalQueue.push(initialConditions.goals[i]);
			
			SourceTargetCouple couplePtr;
			couplePtr.SourceID = id();
			couplePtr.SourceName = initialConditions.name;
			couplePtr.TargetName = initialConditions.goals[i].targetName;

			FleeCouple.insert(couplePtr);

		}

		else {
			//hold on throwing an exception
			std::cout << "here they throw an exception for unsupported goal types" << std::endl;
			throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}
	
	runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	* Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	* And that _waypoints is not empty
	*/
	Util::Vector goalDirection;
	if (!_midTermPath.empty())
	{
		this->updateLocalTarget();
		goalDirection = normalize(this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize(_goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
		(
		(
			(
				Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
				PERFERED_SPEED
				)
			- velocity()
			)
			/
			_SocialForcesParams.sf_acceleration
			)
		*
		MASS;

	// _velocity = _prefVelocity;

#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << _prefVelocity <<
		" and current velocity is: " << _velocity << " or " << velocity() << std::endl;
#endif



	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// _gEngine->addAgent(this, rvoModule);
	assert(_forward.length() != 0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);



	// 
	//DBG - these are dynamic goal sets
	/*
	std::cout
		<< "reset() --- Sets output"
		<< std::endl;

	std::set<SourceTargetCouple>::iterator it;
	for (it = SeekCouple.begin(); it != SeekCouple.end(); it++)
	{
		std::cout
			<< "Seek:it->SourceID: " << it->SourceID
			<< " | it->SourceName: " << it->SourceName
			<< " | it->TargetID: " << it->TargetID
			<< " | it->TargetName: " << it->TargetName
			<< std::endl;
	}

	std::set<SourceTargetCouple>::iterator it2;
	for (it2 = FleeCouple.begin(); it2 != FleeCouple.end(); it2++)
	{
		std::cout
			<< "Flee:it2->SourceName: " << it2->SourceID  
			<< " | it2->SourceName: " << it2->SourceName
			<< " | it2->TargetID: " << it2->TargetID
			<< " | it2->TargetName: " << it2->TargetName
			<< std::endl;
	}
	std::set<AgentColorEntry>::iterator it3;
	for (it3 = AgentColorTable.begin(); it3 != AgentColorTable.end(); it3++)
	{
		std::cout
			<< "Flee:it3->SourceName: " << it3->sourceName
			<< " | it3->id: " << it3->agentID
			<< " | it3->color: " << it3->sourceColor
			<< std::endl;
	}
	*/
 

}



void SocialForcesAgent::calcNextStep(float dt)
{

}

std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
	// Return minimum distance between line segment vw and point p
	float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
	if (lSq == 0.0)
		return std::make_pair((p - l2).length(), l1);   // l1 == l2 case
														// Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
														// We find projection of point p onto the line.
														// It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
	const float t = dot(p - l1, l2 - l1) / lSq;
	if (t < 0.0)
	{
		return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
	}
	else if (t > 1.0)
	{
		return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
	}
	const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
	return std::make_pair((p - projection).length(), projection);
}


Util::Vector SocialForcesAgent::calcProximityForce(float dt)
{
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface * tmp_agent;
	SteerLib::ObstacleInterface * tmp_ob;
	Util::Vector away = Util::Vector(0, 0, 0);
	Util::Vector away_obs = Util::Vector(0, 0, 0);

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
		// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ((*neighbour)->isAgent())
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);

			// direction away from other agent
			Util::Vector away_tmp = normalize(position() - tmp_agent->position());
			// std::cout << "away_agent_tmp vec" << away_tmp << std::endl;
			// Scale force
			// std::cout << "the exp of agent distance is " << exp((radius() + tmp_agent->radius()) -
			//	(position() - tmp_agent->position()).length()) << std::endl;


			// away = away + (away_tmp * ( radius() / ((position() - tmp_agent->position()).length() * B) ));
			away = away +
				(
					away_tmp
					*
					(
						_SocialForcesParams.sf_agent_a
						*
						exp(
						(
							(
							(
								this->radius()
								+
								tmp_agent->radius()
								)
								-
								(
									this->position()
									-
									tmp_agent->position()
									).length()
								)
							/
							_SocialForcesParams.sf_agent_b
							)
						)


						)
					*
					dt
					);
			/*
			std::cout << "agent " << this->id() << " away this far " << away <<
			" distance " << exp(
			(
			(
			(
			radius()
			+
			tmp_agent->radius()
			)
			-
			(
			position()
			-
			tmp_agent->position()
			).length()
			)
			/
			_SocialForcesParams.sf_agent_b
			)
			) << std::endl;
			*/
		}
		else
		{
			// It is an obstacle
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			CircleObstacle * obs_cir = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if (obs_cir != NULL && USE_CIRCLES)
			{
				// std::cout << "Found circle obstacle" << std::endl;
				Util::Vector away_tmp = normalize(position() - obs_cir->position());
				away = away +
					(
						away_tmp
						*
						(
							_SocialForcesParams.sf_wall_a
							*
							exp(
							(
								(
								(
									this->radius()
									+
									obs_cir->radius()
									)
									-
									(
										this->position()
										-
										obs_cir->position()
										).length()
									)
								/
								_SocialForcesParams.sf_wall_b
								)
							)


							)
						*
						dt
						);
			}
			else
			{
				Util::Vector wall_normal = calcWallNormal(tmp_ob);
				std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance

				Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);
				// std::cout << "away_obs_tmp vec" << away_obs_tmp << std::endl;
				// away_obs = away_obs + ( away_obs_tmp * ( radius() / ((position() - min_stuff.second).length() * B ) ) );
				away_obs = away_obs +
					(
						away_obs_tmp
						*
						(
							_SocialForcesParams.sf_wall_a
							*
							exp(
							(
								(
								(this->radius()) -
									(
										this->position()
										-
										min_stuff.second
										).length()
									)
								/
								_SocialForcesParams.sf_wall_b
								)
							)
							)
						*
						dt
						);
			}
		}

	}
	return away + away_obs;
}

Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
		(_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	return calcWallRepulsionForce(dt) + (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}

Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt)
{

	Util::Vector agent_repulsion_force = Util::Vector(0, 0, 0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		(this));

	SteerLib::AgentInterface * tmp_agent;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
		// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ((*neighbour)->isAgent())
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if ((id() != tmp_agent->id()) &&
			(tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001)
			)
		{
			agent_repulsion_force = agent_repulsion_force +
				(tmp_agent->computePenetration(this->position(), this->radius()) * _SocialForcesParams.sf_agent_body_force * dt) *
				normalize(position() - tmp_agent->position());
			// normalized tangential force
			/*
			agent_repulsion_force = agent_repulsion_force +
			(
			(
			(-1*position()) - tmp_agent->position()
			)
			/
			(
			(-1*position()) - tmp_agent->position()
			).length()

			)*0.2;
			*/
			//TODO this can have some funny behaviour is velocity == 0
			Util::Vector tangent = cross(cross(tmp_agent->position() - position(), velocity()),
				tmp_agent->position() - position());
			tangent = tangent / tangent.length();
			float  tanget_v_diff = dot(tmp_agent->velocity() - velocity(), tangent);
			// std::cout << "Velocity diff is " << tanget_v_diff << " tangent is " << tangent <<
			//	" velocity is " << velocity() << std::endl;
			agent_repulsion_force = agent_repulsion_force +
				(_SocialForcesParams.sf_sliding_friction_force * dt *
				(
					tmp_agent->computePenetration(this->position(), this->radius())
					) * tangent * tanget_v_diff

					);
		}

	}
	return agent_repulsion_force;
}

Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{

	Util::Vector wall_repulsion_force = Util::Vector(0, 0, 0);


	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::ObstacleInterface * tmp_ob;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
		// for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = _neighbors.begin();  tmp_o != _neighbors.end();  tmp_o++)
	{
		if (!(*neighbour)->isAgent())
		{
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if (tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001)
		{
			CircleObstacle * cir_obs = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if (cir_obs != NULL && USE_CIRCLES)
			{
				// std::cout << "Intersected circle obstacle" << std::endl;
				Util::Vector wall_normal = position() - cir_obs->position();
				// wall distance
				float distance = wall_normal.length() - cir_obs->radius();

				wall_normal = normalize(wall_normal);
				wall_repulsion_force = wall_repulsion_force +
					((
					(
						(
							wall_normal
							)
						*
						(
							radius() +
							_SocialForcesParams.sf_personal_space_threshold -
							(
								distance
								)
							)
						)
						/
						distance
						)* _SocialForcesParams.sf_body_force * dt);

				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
				// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
				// std::endl;
				wall_repulsion_force = wall_repulsion_force +
					(
						dot(forward(), rightSideInXZPlane(wall_normal))
						*
						rightSideInXZPlane(wall_normal)
						*
						cir_obs->computePenetration(this->position(), this->radius())
						)* _SocialForcesParams.sf_sliding_friction_force * dt;

			}
			else
			{
				Util::Vector wall_normal = calcWallNormal(tmp_ob);
				std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance
				wall_repulsion_force = wall_repulsion_force +
					((
					(
						(
							wall_normal
							)
						*
						(
							radius() +
							_SocialForcesParams.sf_personal_space_threshold -
							(
								min_stuff.first
								)
							)
						)
						/
						min_stuff.first
						)* _SocialForcesParams.sf_body_force * dt);
				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
				// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
				// std::endl;
				wall_repulsion_force = wall_repulsion_force +
					(
						dot(forward(), rightSideInXZPlane(wall_normal))
						*
						rightSideInXZPlane(wall_normal)
						*
						tmp_ob->computePenetration(this->position(), this->radius())
						)* _SocialForcesParams.sf_sliding_friction_force * dt;
			}
		}

	}
	return wall_repulsion_force;
}

std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if (normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin, 0, box.zmax), Util::Point(box.xmax, 0, box.zmax));
		// Ended here;
	}
	else if (normal.z == -1)
	{
		return std::make_pair(Util::Point(box.xmin, 0, box.zmin), Util::Point(box.xmax, 0, box.zmin));
	}
	else if (normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax, 0, box.zmin), Util::Point(box.xmax, 0, box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin, 0, box.zmin), Util::Point(box.xmin, 0, box.zmax));
	}
}

/**
* Basically What side of the obstacle is the agent on use that as the normal
* DOES NOT SUPPORT non-axis-aligned boxes
*
*
* 			   \		   /
* 				\		  /
* 				 \	 a	 /
*				  \		/
* 					 _
* 			a		| |       a
* 					 -
* 				  /     \
* 				 /   a   \
* 				/	      \
* 			   /	       \
*
*
*/
Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if (position().x > box.xmax)
	{
		if (position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if (position().z < box.zmin)
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if (position().x < box.xmin)
	{
		if (position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if (position().z < box.zmin)
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if (position().z > box.zmax)
		{
			return Util::Vector(0, 0, 1);
		}
		else if (position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal(obs);
		}
	}

}

/**
* Treats Obstacles as a circle and calculates normal
*/
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax + box.xmin) / 2, (box.ymax + box.ymin) / 2,
		(box.zmax + box.zmin) / 2);
	return normalize(position() - obs_centre);
}


Vector SocialForcesAgent::Queueing()
{

	Vector _acceleration;  _acceleration.zero();
	Vector qa;
	Point ahead;

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SocialForcesAgent * tmp_agent;
	//init vars for queuing
	Vector tot_vel, avg_vel; tot_vel.zero();  avg_vel.zero();
	int agent_count = 0;

	// get a queue ahead vector by normalizing velocity
	qa = normalize(_velocity);
	// scale qa
	qa = qa * MAX_QUEUE_AHEAD;
	ahead = position() + qa;
	

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();
		neighbor != _neighbors.end(); neighbor++)

	{

		tmp_agent = dynamic_cast<SocialForcesAgent*> (*neighbor);

		if ((*neighbor)->isAgent() && 
			((*tmp_agent).position().vector() - ahead.vector()).norm() <= AHEAD_RADIUS)		
			_acceleration = (-.8f) * _velocity;		
	}
	return _acceleration;
}


bool SocialForcesAgent::lineIntersectsCircle(Point ahead, Point ahead2,Point obstacle) {
	float d1, d2, d3, d4;
	Point aheadTip, ahead2Tip;
	Point obstacleCentre = obstacle;
	const float obst_radius = 05.f;


	aheadTip = position() + ahead;
	ahead2Tip = position() + ahead2;
	d1 = distanceBetween(obstacleCentre, aheadTip);
	d3 = distanceBetween(obstacleCentre, ahead2Tip);
	return (d1 <= d2 || d3 <= d2);
}

Vector SocialForcesAgent::CollissionAvoidance()
{
	Vector _acceleration;  _acceleration.zero();
	Vector qa, qa2, qa3;
	Point ahead, ahead1, ahead2;
	AgentInterface * obstacle;
	AgentInterface * tmp_obstacle;
	Vector cAcceleration;

	//std::cout << "1.0-Entering CollisionForces: " << std::endl;

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));
	//std::cout << "2.0-Entering CollisionForces: " << std::endl;

	//calculate ahead vectors
	ahead = position() + (normalize(velocity()) * MAX_SEE_AHEAD);
	ahead2 = position() + (normalize(velocity()) * MAX_SEE_AHEAD * 0.5);
	//std::cout << "2.5-Entering CollisionForces: " << std::endl;

	float min_distance = 1000.0f;
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();
		neighbour != _neighbors.end(); neighbour++)
	{
		if ((*neighbour)->isAgent())
		{
			/*
			std::cout << "2.6-Entering CollisionForces: "
				<< " | min_dist: " << min_distance
				<< std::endl;
				*/
			tmp_obstacle = dynamic_cast<SocialForcesAgent*> (*neighbour);
			if (distanceBetween(position(), (*tmp_obstacle).position()) < min_distance
				&& lineIntersectsCircle(ahead, ahead2, (*tmp_obstacle).position()))
			{
				/*
				std::cout << "2.7-Entering CollisionForces: "
					<< " | min_dist: " << min_distance
					<< std::endl;
				*/
				obstacle = tmp_obstacle;
			}
			else
				cAcceleration.zero();
		}
	}// most dangerous obstacle is in obstacle object
	if (min_distance != 1000)
	{
		/*
		std::cout << "3.0-Entering CollisionForces: "
			<< " | ahead: " << ahead
			<< " | obst.position: " << (*obstacle).position()
			<< std::endl;
		*/
		//calculate avoidance
		cAcceleration = ahead - (*obstacle).position();
		//std::cout << "4.0-Entering CollisionForces: " << std::endl;
		cAcceleration = normalize(cAcceleration);
		//std::cout << "5.0-Entering CollisionForces: " << std::endl;

		return _acceleration;
	}
	else
	{
		_acceleration.zero();
		return _acceleration;
	}
}



void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	Util::Vector goalDirection;
	goalDirection.zero();
	Util::AutomaticFunctionProfiler profileThisFunction(&SocialForcesGlobals::gPhaseProfilers->aiProfiler);
	Vector qAcceleration, tAcceleration, cAcceleration;
	//DBG
	/*
	std::cout << std::endl;
	int curragentID = id();
	std::cout << "Entering update for agent id#: " << id()
		<< std::endl;
	*/

	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();


	//if midtermpath is not empty and target location is not in line of sight 
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	Vector collavvoidGoal;
	float smallestdistance = 1000, distance;
	AgentInitialConditions ic;

	switch (goalInfo.goalType)

	{
	case 6:

		break;
	case 0: //updating standard static goal
	{
		if (!_midTermPath.empty() && (!this->hasLineOfSightTo(goalInfo.targetLocation)))
		{
			if (reachedCurrentWaypoint())
			{
				this->updateMidTermPath();
			}

			this->updateLocalTarget();

			goalDirection = normalize(_currentLocalTarget - position());

		}
		else
		{
			goalDirection = normalize(goalInfo.targetLocation - position());

		}

	} break; //end case 0

	case 2: {
		//scan spacial database for agent with name  goalInfo.targetName
			getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
				_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

			//load all current neighbors in neighborsAgentInterface
			AgentInterface * tmp_agent;
			std::vector<AgentInterface *> neighborsAgentInterface;
			for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();
				neighbour != _neighbors.end(); neighbour++)
			{
				if ((*neighbour)->isAgent())
				{
					tmp_agent = dynamic_cast<AgentInterface *> (*neighbour);
					neighborsAgentInterface.push_back(tmp_agent);
				}
			}//end creating neighborsAgentInterface list 

			//list all neighborsAgentInterface
			for (std::vector<SteerLib::AgentInterface *>::iterator currAgentInterface = neighborsAgentInterface.begin();
				currAgentInterface != neighborsAgentInterface.end(); currAgentInterface++)
			{
				Util::Color curColor;
				std::set<AgentColorEntry>::iterator currAgentColorEntry;
				//_currentGoal.targetName
				/*
				std::cout << "2.0 - checking case 2: "
					<< " | currColor: " << _color //this is the color of the source agent
					<< " | _currentGoal.targetName: " << _currentGoal.targetName
					<< " | name: " << (*this).getAgentConditions(this).name
												  //<< " | neighColor: " << (*currAgentInterface)->getAgentConditions(*currAgentInterface).color
					<< std::endl;*/
					//yellow (1,1,0) is for seeking

				if ( 
					_color.r == gYellow.r && // gYellow(1.0f, 1.0f, 0.0f)
					_color.g == gYellow.g &&
					_color.b == gYellow.b
					
					)
				{ 
					Point targetPosition = tmp_agent->position();
					Point currentAgentPosition = position();
					//here is the seeking force
					goalDirection = normalize(targetPosition - currentAgentPosition);
					break;
				}
			} //end for loop iterating thru neighborsAgentInterface
		
	} break;// end case 2:
	case 3: {
		//scan spacial database for agent with name  goalInfo.targetName
		getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
			_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
			_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
			_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
			_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
			dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

		//load all current neighbors in neighborsAgentInterface
		AgentInterface * tmp_agent;
		std::vector<AgentInterface *> neighborsAgentInterface;
		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();
			neighbour != _neighbors.end(); neighbour++)
		{
			if ((*neighbour)->isAgent())
			{
				tmp_agent = dynamic_cast<AgentInterface *> (*neighbour);
				neighborsAgentInterface.push_back(tmp_agent);
			}
		}//end creating neighborsAgentInterface list 

		 //list all neighborsAgentInterface
		for (std::vector<SteerLib::AgentInterface *>::iterator currAgentInterface = neighborsAgentInterface.begin();
			currAgentInterface != neighborsAgentInterface.end(); currAgentInterface++)
		{
			Util::Color curColor;
			std::set<AgentColorEntry>::iterator currAgentColorEntry;

			std::cout << "3.0 - checking case 3: "
				<< " | currColor: " << _color
				<< std::endl;
			//grey (.3,.3,.3) is for avoiding
			if (
				_color.r == gGray30.r &&
				_color.g == gGray30.g &&
				_color.b == gGray30.b
				)
			{
				//curColor = (*currAgentColorEntry).sourceColor;
				Point targetPosition = tmp_agent->position();
				Point currentAgentPosition = position();
				//here is the fleeing force
				goalDirection = normalize(targetPosition - currentAgentPosition);
				goalDirection = -goalDirection;
				break;
			}

		} //end for loop iterating thru neighborsAgentInterface
	} break; // end case3
	}

	Util::Vector prefForce = (((goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration / dt)); //assumption here
	prefForce = prefForce + velocity();

	Util::Vector repulsionForce = calcRepulsionForce(dt);
	if (repulsionForce.x != repulsionForce.x)
	{
		std::cout << "Found some nan" << std::endl;
		repulsionForce = velocity();
		// throw GenericException("SocialForces numerical issue");
	}
	Util::Vector proximityForce = calcProximityForce(dt);

	qAcceleration.zero();
	
	//qAcceleration = Queueing();
	//tAcceleration = ((prefForce + repulsionForce + proximityForce+ (qAcceleration* IND_BEH_COEFF)) / (MASS * AGENT_ACC_COEF));
		//+ (qAcceleration* IND_BEH_COEFF);
	/*std::cout
		<< "--Q6 - after accel addition: qaccel" << qAcceleration
		<< " | taccel: " << tAcceleration
		<< std::endl;
		*/
	// #define _DEBUG_ 1
#ifdef _DEBUG_

	if (repulsionForce.length() != 0)
		//DBG std::cout << "agent " << id() << " repulsion force " << repulsionForce << std::endl;
		if (proximityForce.length() != 0)
			//DBG std::cout << "agent " << id() << " proximity force " << proximityForce << std::endl;
			// std::cout << "agent " << id() << " pref force " << prefForce << std::endl;
		// _velocity = _newVelocity;
#endif

			int alpha = 1;
	if (repulsionForce.length() > 0.0)
	{
		alpha = 0;
	}
	/*
	std::cout 
		<< "--Q7:_velocity" << _velocity 
		<< " | _velocity()" << velocity()
		<< " | tAcceleration" << tAcceleration
		<< std::endl;
	*/
	//add steering 
	_velocity = velocity() + tAcceleration;
	/*
	std::cout
		<< "--Q8:_velocity" << _velocity
		<< std::endl;
	*/
	//adding Queue steering to the velocity vector
	cAcceleration = CollissionAvoidance();

	_velocity = (prefForce)+repulsionForce + proximityForce + 
		(qAcceleration*QUEUEING_COEF) + (cAcceleration*COLL_AVOID_COEF);
	// _velocity = (prefForce);
	// _velocity = velocity() + repulsionForce + proximityForce;

	_velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
	_velocity.y = 0.0f;
#ifdef _DEBUG_

	std::cout << "agent " << id() << " speed is " << velocity().length() << "\n | velocity is: " << velocity() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	* Or when the agent is removed for example its true location will not reflect its location in the grid database.
	* Not only that but this error will appear random depending on how well the agent lines up with the grid database
	* boundaries when removed.
	*/
	// std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->updateObject(this, oldBounds, newBounds);

	/*
	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius()*WAYPOINT_THRESHOLD_MULTIPLIER)
	{
	_waypoints.erase(_waypoints.begin());
	}
	*/
	/*
	* Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
	*/
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
		(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
			Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
				goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if (velocity().lengthSquared() > 0.0)
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);

}




void SocialForcesAgent::draw()
{
#ifdef ENABLE_GUI
	AgentInterface::draw();
	// if the agent is selected, do some annotations just for demonstration

#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x - (this->_radius * 3), _position.x + (this->_radius * 3),
		_position.z - (this->_radius * 3), _position.z + (this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); neighbor++)
	{
		if ((*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
				this->position()
				+
				(
				(
					dynamic_cast<AgentInterface*>(*neighbor)->position()
					-
					this->position()
					)
					/ 2), Util::Vector(1, 0, 0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while (__oldPositions.size() > mostPoints)
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size() - 1; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q - 1), gBlack, q / (float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i = 0; (_waypoints.size() > 1) && (i < (_waypoints.size() - 1)); i++)
	{
		if (_gEngine->isAgentSelected(this))
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i + 1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i = 0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1, 0, 0), 0.34f, gBlue);
	}

	for (int i = 0; (_midTermPath.size() > 1) && (i < (_midTermPath.size() - 1)); i++)
	{
		if (_gEngine->isAgentSelected(this))
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i + 1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget + Util::Vector(0, 0.001, 0), Util::Vector(1, 0, 0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();

	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
	Util::Vector normal = calcWallNormal( *tmp_o );
	std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
	Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
	(line.first.z+line.second.z)/2);
	DrawLib::drawLine(midpoint, midpoint+normal, gGreen);

	// Draw the closes point as well
	std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
	DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}
