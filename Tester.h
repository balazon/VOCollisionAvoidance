#pragma once

#include "ORCASolver.h"

#include <vector>

struct Test
{
	std::vector<Agent> agents;
	std::map<int, int> neighbours;
	int testedAgent;
	float Vnewx;
	float Vnewy;
	
};


struct ContinuousTest
{
	std::vector<Agent> agents;
	std::map<int, int> neighbours;
	
	std::vector<float> DestX;
	std::vector<float> DestY;
	
	float timeGiven;//in seconds
	int testedAgent;
};

class Tester
{
public:
	Tester();
	~Tester();
	
	void InitTests();
	void AddTest(Test t);
	void RunTests();
	
	void AddTest(ContinuousTest t);
	
	
private:
	
	std::vector<Test> tests;
	
	std::vector<ContinuousTest> contTests;
	
	ORCASolver solver;
	
	//true for success
	bool RunContinuousTest(ContinuousTest t);
};


