//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	struct timeCompare {
		bool operator()(const CurvePoint& p1, const CurvePoint& p2) const {
			return p1.time < p2.time;
		}
	};
	//debug print
	//DEBUG
	std::cout << "Print controlPoints elements before sorting" << std::endl;
	for (int y = 0; y < controlPoints.size(); y++)
	{
		std::cout << "ControlPoint index: " << y << ";ControlPoint time value: " << controlPoints[y].time << std::endl;
	}


	//Inline sorting of the controlPoints vector on time property
	std::sort(controlPoints.begin(), controlPoints.end(), timeCompare());

	//Post order check
	//DEBUG 
	std::cout << "Print controlPoints elements post sorting" << std::endl;
	for (int y = 0; y < controlPoints.size(); y++)
	{
		//DEBUG
		std::cout << "ControlPoint index: " << y << ";ControlPoint time value: " << controlPoints[y].time << std::endl;
	}

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	//verity that the curve has at least 2 points
	float lastTime = 0;
	if (controlPoints.size() <= 2) {
		std::cerr << "ERROR>>>>Less than two control points in the curve!" << std::endl;

		return false;
	}
	//verify no two consecutive points with same time - remming for now even though it is a problem
	//or is it OK?
	//disabling the check below as per Fernando's post on the class forums
	//https://sakai.rutgers.edu/portal/site/405ae60f-b8c0-4d79-9972-ccc1d3789bdf/tool/2969585d-e153-4db7-b1c1-89624feb4591/discussionForum/message/dfViewThread

	/*
	for (int i = 0; i < controlPoints.size(); i++) {
		//DEBUG std::cout << "ControlPoint index: " << i << ";ControlPoint time value: " << controlPoints[i].time << std::endl;
		if (controlPoints[i].time = lastTime) {
			std::cerr << "ERROR>>>>Two points with same time on the controlPoints vector! " << i << " " << controlPoints[i].time << " "
				<< lastTime << std::endl;
			return false;
		}
		lastTime = controlPoints[i].time;
	}
	*/

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	//Need iterator here
	std::vector<CurvePoint>::iterator pointIterator;

	//vector element counter
	int i = 0;
	//DEBUG std::cout << std::endl;bstd::cout << "Entering the findTimeInterval() function" << "; Time is: " << time << std::endl;

	//Iterate over the controlPoints vector to locate item with time
	for (pointIterator = controlPoints.begin(); pointIterator < controlPoints.end(); pointIterator++, i++) {
		//What if there are duplicate items with the same value of time? 
		//DEBUG std::cout << "probing point index: " << i << " with properties: " << controlPoints[i].time << std::endl;
		
		if (controlPoints[i].time < time && time < controlPoints[i+1].time)  {
			//returning the index of the next point on the curve
			//DEBUG std::cout << "Index of the next point on the curve is: " << i+1 << std::endl;
			nextPoint = i+1;
			return true;
		}
		
	}
	return false; //error locating the point
}


// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	//DEBUG  
	std::cout << std::endl; std::cout << "Entering the useHermiteCurve() function" << "; Time is: " << time << std::endl;
	//DEBUG  
	std::cout << "next point index: " << nextPoint << " ; Time: " << time << std::endl;

	//find previous point
	//normalize time
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	//DEBUG 
	std::cout << "Time interval: " << intervalTime << std::endl;

	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;
	//DEBUG 
	std::cout << "Normal time : " << normalTime << std::endl;

	//calculate newPosition
	//returm newPosition

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useHermiteCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Calculate position at t = time on Hermite curve

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Calculate position at t = time on Catmull-Rom curve

	// Return result
	return newPosition;
}