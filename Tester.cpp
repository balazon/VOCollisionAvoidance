#include "Tester.h"

#include <iostream>

#include <cmath>
#define EPS 0.001

Tester::Tester()
{
}

Tester::~Tester()
{
}

/*
 * help for agent data
Agent:
	float x, y;
	float vx, vy;
	float r;
	float vx_pref;
	float vy_pref;
	float vx_new;
	float vy_new;
	int nearbyAgents[MAXNEARBY];
	float ux[MAXNEARBY];
	float uy[MAXNEARBY];
*/
void Tester::InitTests()
{
	std::vector<Agent> testAgents;
	std::map<int, int> neighbours;
	
	//test 1
	Agent a1 = Agent{0.f, 0.f, 5.5f, 3.f, 2.f, 5.5f, 3.f};
	Agent b1 = Agent{14.f, 10.f, 0.f, 0.f, 8.f, 0.f, 0.f};
	//first agent can see the second (but it is bidirectional for now)
	neighbours.clear();
	neighbours[0] = 1;
	
	testAgents.clear();
	testAgents.push_back(a1);
	testAgents.push_back(b1);
	
	Test t1{testAgents, neighbours, 0, 4.75f, 2.f};
	AddTest(t1);
	
	//test 2
	Agent a2 = Agent{0.f, 0.f, 5.5f, 3.f, 2.f, 5.5f, 3.f};
	Agent b2 = Agent{14.f, 10.f, 0.f, 0.f, 8.f, 0.f, 0.f};
	neighbours.clear();
	neighbours[0] = 1;
	
	testAgents.clear();
	testAgents.push_back(a2);
	testAgents.push_back(b2);
	Test t2{testAgents, neighbours, 0, 9.f, 2.f};
	AddTest(t2);
}

void Tester::AddTest(Test t)
{
	tests.push_back(t);
}


void Tester::RunTests()
{
	int testCount = tests.size();
	int passedTest = 0;
	int testIndex = 0;
	for(Test t : tests)
	{
		testIndex++;
		
		solver.ClearAgents();
		std::vector<int> agentIds;
		for(int i = 0; i < t.agents.size(); i++)
		{
			agentIds.push_back(solver.AddAgent());
		}
		for(auto it = t.neighbours.begin(); it != t.neighbours.end(); it++)
		{
			int i = it->first;
			int j = it->second;
			Agent& a = solver.GetAgent(agentIds[i]);
			Agent& b = solver.GetAgent(agentIds[j]);
			SetAgentsNearby(agentIds[i], agentIds[j]);
		}
		solver.ComputeNewVelocities();
		
		Agent testedAgent = solver.GetAgent(agentIds[t.testedAgent]);
		
		std::cout << "Test " << testIndex << ": " << t.Vnewx << "," << t.Vnewy << "\n";
		std::cout << "Solver res: " << testedAgent.vx_new  << "," << testedAgent.vy_new << "\n\n";
		if(fabs(testedAgent.vx_new - t.Vnewx) < EPS && fabs(testedAgent.vy_new - t.Vnewy) < EPS)
		{
			passedTest++;
		}
	}
	
	
	std::cout << passedTest << " out of " << testCount << " tests passed : ";
	if(passedTest == testCount)
	{
		std::cout << "PASS";
	}
	else
	{
		std::cout << "FAIL";
	}
	std::cout << std::endl;

}