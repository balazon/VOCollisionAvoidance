#pragma once

#include <vector>
#include <unordered_set>

//CPLP for Closest Point Linear Programming
class CPLPSolver
{
public:
	CPLPSolver();
	~CPLPSolver();
	
	void Reset();
	
	//Ax + By < C
	void AddConstraintLinear(float A, float B, float C);
	
	void AddConstraintCircle(float U, float V, float R);
	
	//Destination coordinates
	void SetDestination(float u, float v);
	
	bool HasSolution();
	void Solve(float& resX, float& resY);
	
private:
	//destination coordinates
	float u, v;
	
	enum ConstraintType {CT_LINEAR, CT_CIRCLE};
	
	//3 elements define a constraint
	std::vector<float> constraints;
	
	std::vector<ConstraintType> constraintTypes;
	
	bool feasible;
	
	std::vector<int> order;
	
	std::unordered_set<int> filter;
	
	//nth constraint in array (not in the random order)
	bool pointSatisfiesConstraint(float tx, float ty, int n);
	
	//with regard to random order
	bool pointSatisfiesConstraints(float tx, float ty, int n);
	
	void createRandomOrder();
};


