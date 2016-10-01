#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions

bool calculateAABB(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {

	//get shapeA
	//get shapeB
	int counter = 0;
	std::vector<Util::Vector> ShapeAPtr, ShapeBPtr;
	ShapeAPtr = _shapeA;
	int vectorASize = ShapeAPtr.capacity();
	int vectorBSize = ShapeBPtr.capacity();
	Util::Point max, min;
	max.x = -1E+37f; max.y = -1E+37f; max.z = -1E+37f;
	min.x = 1E+37f; min.y = 1E+37f; min.z = 1E+37f;
	//std::cout << max.x << "," << min.x << std::endl;

	float
		shapeAmaxX = max.x, shapeAmaxZ = max.z,
		shapeAminX = min.x, shapeAminZ = min.z;
	//Print shapeA and shapeB vertices and get shapeAmaxX and shapeBmaxZ
	//std::cout << "Shape A vertices:" << std::endl;
	for (counter; counter < vectorASize; counter++) {
		float currX = ShapeAPtr.at(counter).x, currZ = ShapeAPtr.at(counter).z;
		
		//std::cout <<"Curr:( "<< currX << "," << currZ << ")" <<std::endl;
		
		//std::cout << "(" << ShapeAPtr.at(counter).x << ", "
		//	<< ShapeAPtr.at(counter).y << ", " << ShapeAPtr.at(counter).z << ")" << std::endl;

		if (currX > shapeAmaxX)
			shapeAmaxX = currX;
		if (currZ > shapeAmaxZ)
			shapeAmaxZ = currZ;
		if (currX < shapeAminX)
			shapeAminX = currX;
		if (currZ < shapeAminZ)
			shapeAminZ = currZ;
		/*
		std::cout << "AMax :( " << shapeAmaxX << "," << shapeAmaxZ << ")" << std::endl;
		std::cout << "AMin :( " << shapeAminX << "," << shapeAminZ << ")" << std::endl;
		std::cout << "Curr:( " << currX << "," << currZ << ")" << std::endl;
		*/
	}

	counter = 0;
	ShapeBPtr = _shapeB;
	vectorBSize = ShapeBPtr.capacity();
	float
	shapeBmaxX = max.x, shapeBmaxZ = max.z,
	shapeBminX = min.x, shapeBminZ = min.z;
	//std::cout << "Shape B vertices:" << std::endl;
	for (counter; counter < vectorBSize; counter++) {
		float currX = ShapeBPtr.at(counter).x, currZ = ShapeBPtr.at(counter).z;
		/*
		std::cout << "Curr:( " << currX << "," << currZ << ")" << std::endl;
		*/
		
		//std::cout << "(" << ShapeBPtr.at(counter).x << ", "
		//	<< ShapeBPtr.at(counter).y << ", " << ShapeBPtr.at(counter).z << ")" << std::endl;
		if (currX > shapeBmaxX)
			shapeBmaxX = currX;
		if (currZ > shapeBmaxZ)
			shapeBmaxZ = currZ;
		if (currX < shapeBminX)
			shapeBminX = currX;
		if (currZ < shapeBminZ)
			shapeBminZ = currZ;
		/*
		std::cout << "BMax :( " << shapeBmaxX << "," << shapeBmaxZ << ")" << std::endl;
		std::cout << "BMin :( " << shapeBminX << "," << shapeBminZ << ")" << std::endl;
		std::cout << "Curr:( " << currX << "," << currZ << ")" << std::endl;
		*/
	}
	/*
	std::cout << "Shape A max: (" << shapeAmaxX << ", " << shapeAmaxZ << ")" << std::endl;
	std::cout << "Shape A min: (" << shapeAminX << ", " << shapeAminZ << ")" << std::endl;
	std::cout << "Shape B max: (" << shapeBmaxX << ", " << shapeBmaxZ << ")" << std::endl;
	std::cout << "Shape B min: (" << shapeBminX << ", " << shapeBminZ << ")" << std::endl;
	*/
	//
	if ( 
		(shapeAmaxX < shapeBminX) || //intersect on X
		(shapeAmaxZ < shapeBminZ))   //intersect on Z
		return false; //can intersect

	if ((shapeAminX > shapeBmaxX) || //intersect on X
		(shapeAminZ > shapeBmaxZ))   //intersect on Z
		return false; //can intersect


	return true; //Polygons cannot intersect
}


bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, 
	const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	if (!calculateAABB(_shapeA, _shapeB)) { //if AABB - returns false - skip those polygons
		//std::cout << "Polygons cannot intersect - exiting this set .. " << std::endl;
		return false;
	}
	else
		return true;
	//std::cout << "Those two polygons can intersect." << std::endl;

	//The caller sends permutation of all shapes 
	//We are comparing only two
	//Since we deal with vertices at this point
	//load the 




	return false; // There is no collision
}

