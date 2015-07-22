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


class Tester
{
public:
	Tester();
	~Tester();
	
	void InitTests();
	void AddTest(Test t);
	void RunTests();
	
private:
	
	std::vector<Test> tests;
	ORCASolver solver;
};


