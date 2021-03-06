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


	Point firstPoint, secondPoint;

	float startTime = controlPoints[0].time,
		endTime = controlPoints[controlPoints.size() - 1].time;
	int size = controlPoints.size();
	int agentIndex = 0;

	Point newPosition;
	unsigned int maxPointIndex = controlPoints.size() - 1;
	unsigned int firstNonBoundaryIndex = 0, lastNonBoundaryIndex = maxPointIndex;
	float maxtime = controlPoints[maxPointIndex].time;

	float fineWindow;
	if (window = 5)
		fineWindow = 2;
	else fineWindow = 1;

	for (int timeCounter = startTime; timeCounter < endTime; timeCounter = timeCounter + fineWindow)
	{
		if (calculatePoint(firstPoint, timeCounter) &&
			calculatePoint(secondPoint, timeCounter + fineWindow)
			) {

			DrawLib::drawLine(firstPoint, secondPoint, curveColor, curveThickness);

		}
		else {
		}
	}


	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	unsigned int cpSize = controlPoints.size();


	struct timeCompare {
		bool operator()(const CurvePoint& p1, const CurvePoint& p2) const {
			return p1.time < p2.time;
		}
	};
	/*DEBUG
	for (int y = 0; y < controlPoints.size(); y++)
	{
	std::cout << "ControlPoint index: " << y << ";ControlPoint time value: " << controlPoints[y].time << std::endl;
	}
	*/

	//Inline sorting of the controlPoints vector on time property
	std::sort(controlPoints.begin(), controlPoints.end(), timeCompare());

	//remove extra control point with same time stamps

	for (int i = 1; i < cpSize - 1; i++) {
		if (controlPoints[i - 1].time == controlPoints[i].time) {
			controlPoints[i].position.x = controlPoints[i - 1].position.x;
			controlPoints[i].position.y = controlPoints[i - 1].position.y;
			controlPoints[i].position.z = controlPoints[i - 1].position.z;
			controlPoints[i].tangent.x = controlPoints[i - 1].tangent.x;
			controlPoints[i].tangent.y = controlPoints[i - 1].tangent.y;
			controlPoints[i].tangent.z = controlPoints[i - 1].tangent.z;
		}
	}


	//Post order check
	/*
	for (int y = 0; y < controlPoints.size(); y++)
	{
	//DEBUG
	Point currPos = controlPoints[y].position;
	float currTime = controlPoints[y].time;
	std::cout << "ControlPoint index: " << y << ";ControlPoint time value: "
	<< currTime << " (" << currPos.x << "," << currPos.y << "," << currPos.z << ")"
	<< std::endl;
	}*/

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	//DEBUG  
	//std::cout << std::endl; std::cout << "Entering the calculatePoint() function" << std::endl;

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

	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{

	//verity that the curve has at least 2 points
	float lastTime = 0;
	if (type == hermiteCurve &&  controlPoints.size() <= 2) {
		std::cerr << "ERROR>>>>Less than two control points in the curve - cannot do Hermite spline!" << std::endl;

		return false;
	}
	if (type == catmullCurve &&  controlPoints.size() < 4) {
		std::cerr << "ERROR>>>>Less than four control points in the curve - cannot do Catmull-Rom spline!" << std::endl;

		return false;
	}


	//check no to go over the time


	//verify no two consecutive points with same time - remming for now even though it is a problem
	//or is it OK?
	//disabling the check below as per Fernando's post on the class forums
	//https://sakai.rutgers.edu/portal/site/405ae60f-b8c0-4d79-9972-ccc1d3789bdf/tool/2969585d-e153-4db7-b1c1-89624feb4591/discussionForum/message/dfViewThread

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{

	//Need iterator here
	std::vector<CurvePoint>::iterator pointIterator;

	//vector element counter
	int i = 0;
	float maxtime = controlPoints.at(controlPoints.size() - 1).time;
	if (time > maxtime) {
		return false;
	}



	//Iterate over the controlPoints vector to locate item with time
	for (pointIterator = controlPoints.begin(); pointIterator < controlPoints.end(); pointIterator++, i++) {

		if (controlPoints[i].time <= time && time <= controlPoints[i + 1].time) {
			//returning the index of the next point on the curve
			nextPoint = i + 1;
			return true;
		}

	}

	exit(0);
	return false; //error locating the point
}


// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	//find previous point
	//normalize time
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	//calculate newPosition

	newPosition =
		(
		(
			(2 * pow(normalTime, 3))
			- (3 * pow(normalTime, 2))
			+ 1
			)
			*
			(controlPoints.at(nextPoint - 1).position)
			) +
			(
		(
				(pow(normalTime, 3))
			- (2 * pow(normalTime, 2))
			+ normalTime
			)
				*
				((controlPoints.at(nextPoint - 1).tangent)*(intervalTime))
				) +
				(
		(
					(-2 * pow(normalTime, 3))
			+ (3 * pow(normalTime, 2))
			)
					*
					(controlPoints.at(nextPoint).position)
					) +
					(
		(
						(pow(normalTime, 3))
			- (pow(normalTime, 2))
			)
						*
						((controlPoints.at(nextPoint).tangent)*(intervalTime))
						);


	// Calculate position at t = time on Hermite curve

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	//check robustness

	Point newPosition;
	unsigned int cpSize = controlPoints.size();
	unsigned int maxPointIndex = cpSize - 1;
	unsigned int firstNonBoundaryIndex = 0, lastNonBoundaryIndex = maxPointIndex;
	float maxtime = controlPoints[maxPointIndex].time;
	Point tangent0, tangent1, tangent2, tangent3;

	Point
		p0 = controlPoints[(nextPoint - 2 + cpSize) % cpSize].position,
		p1 = controlPoints[(nextPoint - 1 + cpSize) % cpSize].position,
		p2 = controlPoints[nextPoint].position,
		p3 = controlPoints[(nextPoint - 1 + cpSize) % cpSize].position;

	float	t0 = controlPoints[(nextPoint - 2 + cpSize) % cpSize].time,
		t1 = controlPoints[(nextPoint - 1 + cpSize) % cpSize].time,
		t2 = controlPoints[nextPoint].time,
		t3 = controlPoints[(nextPoint + 1 + cpSize) % cpSize].time,
		dt = t2 - t1;



	//we are at the last point 
	if (nextPoint == maxPointIndex)
		p3 = controlPoints[0].position;

	//we are at the first point
	if (nextPoint == 1)
		p0 = controlPoints[maxPointIndex].position;



	//normalize time
	float normalTime, intervalTime;

	intervalTime = t2 - t1;
	normalTime = (time - t1) / intervalTime;

	//check if we are in 
	//we are either in first or last curve

	newPosition.x = 0.5*(
		(2 * p1.x) +
		((p2.x - p0.x)*normalTime) +
		(((2 * p0.x) - (5 * p1.x) + (p2.x * 4) - p3.x)*normalTime*normalTime) +
		((-1 * p0.x + 3 * p1.x - 3 * p2.x + p3.x)*normalTime*normalTime*normalTime));

	newPosition.y = 0.5*(
		(2 * p1.y) +
		((p2.y - p0.y)*normalTime) +
		(((2 * p0.y) - (5 * p1.y) + (p2.y * 4) - p3.y)*normalTime*normalTime) +
		((-1 * p0.y + 3 * p1.y - 3 * p2.y + p3.y)*normalTime*normalTime*normalTime));

	newPosition.z = 0.5*(
		(2 * p1.z) +
		((p2.z - p0.z)*normalTime) +
		(((2 * p0.z) - (5 * p1.z) + (p2.z * 4) - p3.z)*normalTime*normalTime) +
		((-1 * p0.z + 3 * p1.z - 3 * p2.z + p3.z)*normalTime*normalTime*normalTime));

	// Calculate position at t = time on Catmull-Rom curve

	// Return result

	return newPosition;
}