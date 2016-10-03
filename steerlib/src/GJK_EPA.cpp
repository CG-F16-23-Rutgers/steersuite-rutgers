#include "obstacles/GJK_EPA.h"
#include <algorithm>
#include <vector>


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions

bool SteerLib::GJK_EPA::calculateAABB(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {

	//get shapeA
	//get shapeB
	int counter = 0;
	std::vector<Util::Vector> ShapeAPtr, ShapeBPtr;
	ShapeAPtr = _shapeA;
	size_t vectorASize = ShapeAPtr.capacity();
	size_t vectorBSize = ShapeBPtr.capacity();
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

Util::Vector SteerLib::GJK_EPA::polygonCenter(const std::vector<Util::Vector>& _shape) {
	Util::Vector center;
	float area = 0;
	std::vector<Util::Vector> ShapePtr;
	ShapePtr = _shape;
	size_t vectorSize = ShapePtr.capacity();


	for (int currIndex = 0; currIndex < vectorSize; currIndex++) {
		int nextIndex = (currIndex + 1) % vectorSize;
		area = area + ((ShapePtr.at(currIndex).x)*(ShapePtr.at(nextIndex).z) - (ShapePtr.at(nextIndex).x)*(ShapePtr.at(currIndex).z));
	}
	area = (.5f * area);
	//DBG std::cout << "Area:" << area << std::endl;


	for (int currIndex = 0; currIndex < vectorSize; currIndex++) {
		int nextIndex = (currIndex + 1) % vectorSize;
		center.x = center.x + ((ShapePtr.at(currIndex).x) + (ShapePtr.at(nextIndex).x))
			*((ShapePtr.at(currIndex).x)*(ShapePtr.at(nextIndex).z) - (ShapePtr.at(nextIndex).x)*(ShapePtr.at(currIndex).z));
		center.z = center.z + ((ShapePtr.at(currIndex).z) + (ShapePtr.at(nextIndex).z))
			*((ShapePtr.at(currIndex).x)*(ShapePtr.at(nextIndex).z) - (ShapePtr.at(nextIndex).x)*(ShapePtr.at(currIndex).z));
	}
	center.x = center.x / (6 * area);
	center.z = center.z / (6 * area);
	//DBG std::cout << "Center:(" << center.x << ", " << center.z << ")"<< std::endl;
	return center; //returns polygon center
}

Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& _shapeA, 
	const std::vector<Util::Vector>& _shapeB,
	Util::Vector& DVector) {
	//Add a point to Minkowski Difference
	Util::Vector MinkowskiDiff;
	std::vector<Util::Vector> ShapeAPtr, ShapeBPtr;
	Util::Vector	DVectorPtr, p1Simp, p2Simp;
	ShapeAPtr = _shapeA;
	ShapeBPtr = _shapeB;

	//size_t vectorASize = ShapeAPtr.size();
	//size_t vectorBSize = ShapeBPtr.size();

	p1Simp = _shapeA.at(0); //maxdot
	p2Simp = ShapeBPtr.at(0);
	/*
	std::cout << "  support() - p1Simp :(" << p1Simp.x << ", " << p1Simp.z << ")" << std::endl;
	std::cout << "  support() - p2Simp :(" << p2Simp.x << ", " << p2Simp.z << ")" << std::endl;
	std::cout << "   support() - DVector (" << DVector.x << ", " << DVector.z << ")" << std::endl;
	*/
	//locate the furthest vertex in direction of d

	std::vector<Util::Vector>::iterator it;
	int i = 0;

	for (it = ShapeAPtr.begin(); it < ShapeAPtr.end();it++, i++) {
		/*
		std::cout << "   support() - p1Simp lookup - i: " << i << std::endl;
		std::cout << "   support() - p1Simp lookup - ShapeAPtr (" << ShapeAPtr.at(i).x << ", " << ShapeAPtr.at(i).z << ")" << std::endl;
		std::cout << "   support() - p1Simp lookup - (DVector*ShapeAPtr.at(i) " << DVector*ShapeAPtr.at(i) << std::endl;
		std::cout << "   support() - p1Simp lookup - (DVector*p1Simp " << DVector*p1Simp << std::endl;
		*/
		if (DVector*ShapeAPtr.at(i) < DVector*p1Simp) {
			p1Simp = ShapeAPtr.at(i);
			//std::cout << "  support() - p1Simp (" << p1Simp.x << ", " << p1Simp.z << ")" << std::endl;
		}
	}

	//DVector = -DVector;
	i = 0;
	for (it = ShapeBPtr.begin(); it < ShapeBPtr.end(); it++, i++) {
		/*
		std::cout << "   support() - p2Simp lookup - i: " << i << std::endl;
		std::cout << "   support() - p2Simp lookup - ShapeBPtr (" << ShapeBPtr.at(i).x << ", " << ShapeBPtr.at(i).z << ")" << std::endl;
		std::cout << "   support() - p2Simp lookup - (DVector*ShapeBPtr.at(i) " << DVector*ShapeBPtr.at(i) << std::endl;
		std::cout << "   support() - p2Simp lookup - (DVector*p2Simp " << DVector*p2Simp << std::endl;
		*/
		if (DVector*ShapeBPtr.at(i) > DVector*p2Simp) {
			p2Simp = ShapeBPtr.at(i);
			//std::cout << "  support() - p2Simp (" << p2Simp.x << ", " << p2Simp.z << ")" << std::endl;
		}
	}
	/*
	std::cout << "   support() - p1Simp :(" << p1Simp.x << ", " << p1Simp.z << ")" << std::endl;
	std::cout << "   support() - p2Simp :(" << p2Simp.x << ", " << p2Simp.z << ")" << std::endl;
	*/
	//DVector = -DVector; 

	MinkowskiDiff = p2Simp - p1Simp;
	//DBG std::cout << "    support() - about to add point :(" << MinkowskiDiff.x << ", " << MinkowskiDiff.z << ") to simplex" << std::endl;
	
	return MinkowskiDiff; //choice f d depends from a and b
}

Util::Vector tripleProduct(Util::Vector A, Util::Vector B, Util::Vector C) {
		// (A x B) x C = B (C * A) - A( C * B)
	Util::Vector tripleProduct;
	
	tripleProduct = (B*(C*A)) - (A*(C*B));

	return tripleProduct;
}

bool SteerLib::GJK_EPA::containsOrigin(std::vector<Util::Vector>& _simplex, Util::Vector& DVector) {
	//std::vector<Util::Vector> simplexPtr;
	//simplexPtr =  _simplex;


	//DBG print of simplex and d
	/*
	std::cout << std::endl;	std::cout << std::endl;
	std::vector<Util::Vector>::iterator it;
	std::cout << "containsOrigin() - print all simplex point /it/ - size: " << _simplex.size() << std::endl;
	int i = 0;
	for (it = _simplex.begin(); it < _simplex.end(); it++, i++)
	{
		std::cout << " _simplex "<< i<<" :(" << _simplex.at(i).x << ", " << _simplex.at(i).z << ")" << std::endl;
	}
	std::cout << "containsOrigin() - DVector :(" << DVector.x << ", " << DVector.z << ")" << std::endl;
	*/
	//last entry in the simplex
	Util::Vector a = _simplex.at(_simplex.size() - 1);
	//DBG std::cout << "containsOrigin() - lastSimplex :(" << lastSimplex.x << ", " << lastSimplex.z << ")" << std::endl;
	Util::Vector a0, b, ab, abPerp, c, ac, acPerp;
	//std::cout << "containsOrigin() - lastpoint /a/ on simplex: (" << a.x << ", " << a.z << ")" << std::endl;

	a0 = -a;
	//DBG std::cout << "containsOrigin() - a0 :(" << a0.x << ", " << a0.z << ")" << std::endl;
	if (_simplex.capacity() == 3) { //if simplex triangel
		//std::cout << "      Simplex is a triangle" << std::endl;

		//we already have 3 points
		b = _simplex.at(_simplex.size()-2);
		c = _simplex.at(_simplex.size() - 3); //c is at index 0
		/*
		std::cout << "containsOrigin() - b :(" << b.x << ", " << b.z << ")" << std::endl;
		std::cout << "containsOrigin() - c :(" << c.x << ", " << c.z << ")" << std::endl;
		*/
		//calculate the edges
		ab = b - a;
		ac = c - a;
		//normals
		abPerp = tripleProduct(ac, ab, ab);
		acPerp = tripleProduct(ab, ac, ac);
		//if origin is in R4 -> abPerp*a0 > 0
		//std::cout << "is R4? - (abPerp*a0) > 0 " << acPerp*a0 << std::endl;
		if (abPerp*a0 > 0) { // if (abPerp*a0 > 0)
			//remove c
			for (int i = 0; i < _simplex.size(); i++) {
				if ((_simplex.at(i) == c)) { //we are at c
					std::cout << "REMOVING c" << std::endl;
					_simplex.erase(_simplex.begin() + i);
				}
			}

			//change d abPerp
			DVector = abPerp;
		} else { // else (abPerp*a0 > 0)
			//origin is R3
			//std::cout << "is R3? - (acPerp*a0) > 0 " << acPerp*a0  << std::endl;

			if (acPerp*a0 > 0) { // if (acPerp*a0 > 0)
				//remove point b
				for (int i = 0; i < _simplex.size(); i++) {
					if ((_simplex.at(i) == b)) { //we are at  b
						std::cout << "REMOVING b" << std::endl;
						_simplex.erase(_simplex.begin() + i);
						//do the removal
					}
				}
				std::cout << "size: " << _simplex.size() << "|capacity: " << _simplex.capacity() << std::endl;

				DVector = acPerp;
			} else { // else (acPerp*a0 > 0) 
				//std::cout <<  std::endl;
				/*
				std::cout << "        It is in R5." <<  std::endl;
				std::cout << "        About to return to JGK with true: " << _simplex.size() << std::endl;
				int i = 0;
				std::vector<Util::Vector>::iterator it;
				for (it = _simplex.begin(); it < _simplex.end(); it++, i++)
				{
					std::cout << "        _simplex " << i << " :(" << _simplex.at(i).x << ", " << _simplex.at(i).z << ")" << std::endl;
				}
				*/
				//std::cout << "About to return from containsOrigin() -  returning true - line 278" << std::endl;
				return true;
		
			}
		}
	} else { //else - if simplex triangle - simplex is a line
		std::cout << "      Simplex is a line" << std::endl;
		//it it a line - get(B) - it is second to last point
		b = _simplex.at(_simplex.size()-2);
		//std::cout << "b :(" << b.x << ", " << b.z << ")" << std::endl;

		//get AB
		ab = b - a;
		//calc abPerp
		abPerp = tripleProduct(ab, a0, ab);
		//std::cout << "abPerp:" << abPerp << std::endl;
		//if abPerp is 0 - use the normal
		if (abPerp.x == 0 && abPerp.y == 0 && abPerp.z == 0) {
			abPerp = b - a;
			abPerp = normalize(abPerp);
			//std::cout << "abPerp:" << abPerp << std::endl;

		}

		//set d to abPerp
		DVector = abPerp;
		//std::cout << "d :(" << DVector.x << ", " << DVector.z << ")" << std::endl;

	}
	return false; //simplex does not containt origin 
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB,
	 std::vector<Util::Vector>& _simplex) {
	Util::Vector d;
	Util::Vector c1, c2, tempVect;

	c1 = polygonCenter(_shapeA);
	//DBG 
	//std::cout << " GJK() - Center A:(" << c1.x << ", " << c1.z << ")" << std::endl;
	c2 = polygonCenter(_shapeB);
	//DBG 
	//std::cout << " GJK() - Center B :(" << c2.x << ", " << c2.z << ")" << std::endl;
	d = c1 - c2;
	//DBG std::cout << " GJK() - D :(" << d.x << ", " << d.z << ")" << std::endl;

	//DBG - print tempVect 
	//DBG std::cout << "tempVect :(" << tempVect.x << ", " << tempVect.z << ")" << std::endl;
	
	//enter first Minkowski Difference point
	//DBG std::cout <<  " GJK() - first call to support:(" << d.x << ", " << d.z << ")" << std::endl;
	_simplex.push_back(support(_shapeA, _shapeB, d));
	//negate d
	d = -d;
	//DBG std::cout << "GJK() - D - right after negation:(" << d.x << ", " << d.z << ")" << std::endl;

	int iterationCounter = 0;

	while (true) {

		_simplex.push_back(support(_shapeA, _shapeB, d));
		/*
		//dot product between the last entry and vector d
		std::cout << "last entry: " << _simplex.at(_simplex.size() - 1) << " and vector d" << d <<std::endl;

		std::cout << "dot product between the last entry and vector d: " << _simplex.at(_simplex.size() - 1) * d << std::endl;
		*/
		if (_simplex.at(_simplex.size()-1) * d <= 0) {
			//DBG 
			/*
			std::cout << "About to exit from GJK returning false to CollisionAIModule" << std::endl;
			std::cout << std::endl;
			std::cout << std::endl;
			*/
			return false; //no collision
		} else {
			//std::cout << " GJK() - Calling containsOrigin - iteration: " << iterationCounter << std::endl;

			if (containsOrigin(_simplex, d)) {
				//DBG 
				/*
				std::cout << "About to exit from GJK returning true to CollisionAIModule" << std::endl;
				std::cout << std::endl;
				std::cout << std::endl;
				*/
				return true;  //Origin in simplex - collision 100%
			}
		}
		iterationCounter++;
	}
	return true; //is_colliding
}

Util::Vector SteerLib::GJK_EPA::normalize(Util::Vector& _toNormalize) {
	_toNormalize = _toNormalize / _toNormalize.length();
	return _toNormalize;
}

void SteerLib::GJK_EPA::getNearestEdge(std::vector<Util::Vector>& simplex, float& distance,
	Util::Vector& normal, int& index) {
	distance = FLT_MAX;
	std::vector<Util::Vector>::iterator it;
	Util::Vector v1, v2, edge, originTov1, n;
	float dist;


	int i = 0, j = 0;
	for (it = simplex.begin(); it < simplex.end(); it++, i++) {
		if (i + 1 == simplex.size())
			j = 0;
		else
			j = i + 1;
		v1 = simplex[i];
		v2 = simplex[j];

		edge = v2 - v1;

		originTov1 = v1;

		n = tripleProduct(edge, originTov1, edge);
		n = normalize(n);
		//distance from origin to the edge
		dist = n*v1;

		if (dist < distance) {
			distance = dist;
			index = j;
			normal = n;
		}
	}
	return;
}

void SteerLib::GJK_EPA::EPA(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, 
	std::vector<Util::Vector>& simplex, float& penetration_depth, Util::Vector& penetration_vector) {
	float distance, d;
	int i;
	Util::Vector normal;
	Util::Vector supportvector;
	Util::Vector& sup = supportvector;


	while (true)
	{
		getNearestEdge(simplex, distance, normal, i);
		sup = support(_shapeA, _shapeB, normal);
		d = sup*normal;

		if (d - distance <= 0)
		{
			penetration_vector = normal;
			penetration_depth = distance;
			return;
		}
		else {
			simplex.insert(simplex.begin() + i, sup);
		}
	}

}

bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, 
	const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool is_colliding = false;
	std::vector<Util::Vector> shapePtr;
	shapePtr = _shapeA;


	/*
	std::vector<Util::Vector>::iterator it;
	int i = 0;
	shapePtr = _shapeA;
	std::cout << "shapeA:";
	for (it = shapePtr.begin(); it < shapePtr.end(); it++, i++) {
		std::cout << "(" << shapePtr.at(i).x << ", " << shapePtr.at(i).z << ")|";
	} std::cout << std::endl;
	
	shapePtr = _shapeB;
	i = 0;
	std::cout << "shapeB:";
	for (it = shapePtr.begin(); it < shapePtr.end(); it++, i++) {
		std::cout << "("<<shapePtr.at(i).x << ", " << shapePtr.at(i).z << ")|" ;
	} std::cout << std::endl;
	*/

	//The caller sends completee permutation of all shapes 
	//We are comparing only two
	//Since we deal with vertices at this point

	if (!calculateAABB(_shapeA, _shapeB)) { //if AABB - returns false - skip those polygons
		//DBG std::cout << "intersect() - Polygons cannot intersect - exiting this set .. " << std::endl;
		return false;
	}
	//if calculatwAABB return true
	//will run the GJK algorithm on the polygons
	else {
		is_colliding = GJK(_shapeA, _shapeB, simplex);
		//std::cout << "is_coliding: " << is_colliding  << std::endl;

		//DBG 
		//std::cout << "intersect() - Those two polygons are intersecting." << std::endl;
		if (is_colliding) {
			EPA(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
		}
		return is_colliding; //this probably always be true
	}
	//return false; // There is no collision
	//we shouldn't ever reach here
}

