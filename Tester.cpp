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
	float maxVelocityMagnitude;
	float maxAccMagnitude;
*/
void Tester::InitTests()
{
	std::vector<Agent> testAgents;
	std::vector<std::pair<int, int>> neighbours;
	
	//test 1
	Agent a1 = Agent{0.f, 0.f, 5.5f, 3.f, 2.f, 5.5f, 3.f, 1000.f, 100.f};
	Agent b1 = Agent{14.f, 10.f, 0.f, 0.f, 8.f, 0.f, 0.f, 1000.f, 100.f};
	//first agent can see the second (but it is bidirectional for now)
	neighbours.clear();
	neighbours.push_back({0, 1});
	
	
	testAgents.clear();
	testAgents.push_back(a1);
	testAgents.push_back(b1);
	
	Test t1{testAgents, neighbours, 0, 4.75f, 2.f};
	//AddTest(t1);
	
	//test 2
	Agent a2 = Agent{0.f, 0.f, 9.f, 4.f, 2.f, 9.f, 4.f, 1000.f, 100.f};
	Agent b2 = Agent{14.f, 10.f, 0.f, 0.f, 8.f, 0.f, 0.f, 1000.f, 100.f};
	neighbours.clear();
	neighbours.push_back({0, 1});
	
	testAgents.clear();
	testAgents.push_back(a2);
	testAgents.push_back(b2);
	Test t2{testAgents, neighbours, 0, 9.f, 2.f};
	//AddTest(t2);
	
	//test 3
	Agent a3 = Agent{0.f, 0.f, 0.75f, -3.f, 2.f, 0.f, -6.f, 5.f, 100.f};
	Agent b3 = Agent{14.f, 10.f, -4.75f, -6.f, 8.f, 0.f, 0.f, 1000.f, 100.f};
	neighbours.clear();
	neighbours.push_back({0, 1});
	
	testAgents.clear();
	testAgents.push_back(a3);
	testAgents.push_back(b3);
	
	Test t3{testAgents, neighbours, 0, 0.f, -5.f};
	//AddTest(t3);
	
	//continuous tests:
	std::vector<float> DX, DY;
	//test 4 - 2 agents cross each other's path
	Agent a4 = Agent{100.f, 5.f, 0.f, 0.f, 20.f, 0.f, 0.f, 100.f, 200.f};
	Agent b4 = Agent{-100.f, 0.f, 0.f, 0.f, 20.f, 0.f, 0.f, 100.f, 200.f};
	neighbours.clear();
	neighbours.push_back({0, 1});
	testAgents.clear();
	testAgents.push_back(a4);
	testAgents.push_back(b4);
	DX.clear();DY.clear();
	DX.push_back(-100.f); DY.push_back(5.f);
	DX.push_back(100.f); DY.push_back(0.f);
	ContinuousTest ct4{testAgents, neighbours, DX, DY, 4.f, 0};
	//AddTest(ct4);
	
	
	//svgexport
	//Agent sa1 = Agent(2.f, 2.f, 2.f, 1.f, 1.f, 2.f, 1.f, 5.f, 0.8f);
	Agent sa1 = Agent(2.f, 2.f, 2.f, 1.5f, 1.f, 2.f, 1.f, 5.f, 0.8f);
	Agent sa2 = Agent(8.f, 3.f, -2.f, 3.f, 1.f, -2.f, 1.f, 5.f, 0.8f);
	Agent sa3 = Agent(3.f, 9.f, 0.9f, -2.f, 1.f, 1.f, -2.f, 5.f, 0.8f);
	Agent sa4 = Agent(5.f, 5.f, 0.f, 0.f, 1.f, 1.f, 0.f, 5.f, 0.8f);
	Agent sa5 = Agent(7.f, 8.f, -2.f, -2.f, 1.f, 1.f, 0.f, 5.f, 0.8f);
	
	testAgents.clear();
	testAgents.push_back(sa1);
	testAgents.push_back(sa2);
	testAgents.push_back(sa3);
	testAgents.push_back(sa4);
	testAgents.push_back(sa5);
	
	neighbours.clear();
	for(int i = 0; i < testAgents.size(); i++)
	{
		for(int j = 0; j < testAgents.size(); j++)
		{
			if(i == j)
			{
				continue;
			}
				
			neighbours.push_back({i, j});
		}
	}
	
	Test st{testAgents, neighbours, 3, 0.f, 0.f};
	
	/*RunTest(st);
	SVGExporter::writeUnits("unit01.svg", &solver, testAgents.size());
	SVGExporter::writeUnitORCAs("unitORCA01.svg", &solver, testAgents.size(), 3);
	*/
	
	Agent sa21 = Agent(2.f, 2.f, -2.f, -1.5f, 1.f, 2.f, 1.f, 5.f, 0.8f);
	Agent sa22 = Agent(8.f, 3.f, 2.f, -3.f, 1.f, -2.f, 1.f, 5.f, 0.8f);
	Agent sa23 = Agent(3.f, 9.f, -0.9f, 2.f, 1.f, 1.f, -2.f, 5.f, 0.8f);
	Agent sa24 = Agent(5.f, 5.f, 0.f, 0.f, 1.f, 1.f, 0.f, 5.f, 0.8f);
	Agent sa25 = Agent(7.f, 8.f, 2.f, 2.f, 1.f, 1.f, 0.f, 5.f, 0.8f);
	testAgents.clear();
	testAgents.push_back(sa21);
	testAgents.push_back(sa22);
	testAgents.push_back(sa23);
	testAgents.push_back(sa24);
	testAgents.push_back(sa25);
	
	Test st2{testAgents, neighbours, 3, 0.f, 0.f};
	
	/*RunTest(st2);
	SVGExporter::writeUnits("C:/temp/unit02.svg", &solver, testAgents.size());
	SVGExporter::writeUnitORCAs("unitORCA02.svg", &solver, testAgents.size(), 3);
	*/
	
	//Agent sa31 = Agent(2.f, 2.f, -2.f, -1.5f, 1.f, 2.f, 1.f, 5.f, 0.8f);
	
	float sc=42.f;
	Agent sa31 = Agent(sc * 2.f, sc * 2.f, 0.f, 0.f, sc * 1.f, 0.f, 0.f, 600.f, 2048.f);
	Agent sa32 = Agent(sc * 7.f, sc * 2.f, 0.f, 0.f, sc * 1.f, 0.f, 0.f, 600.f, 2048.f);
	Agent sa33 = Agent(sc * 4.5f, sc * 6.2f, 0.f, 0.f, sc * 1.f, 0.f, 0.f, 600.f, 2048.f);
	
	neighbours.clear();
	neighbours.push_back({0, 1});neighbours.push_back({1, 0});
	neighbours.push_back({0, 2});neighbours.push_back({2, 0});
	neighbours.push_back({1, 2});neighbours.push_back({2, 1});
	
	testAgents.clear();
	testAgents.push_back(sa31);
	testAgents.push_back(sa32);
	testAgents.push_back(sa33);
	DX.clear();DY.clear();
	DX.push_back(sc * 7.f); DY.push_back(sc * 5.f);
	DX.push_back(sc * 2.f); DY.push_back(sc * 5.f);
	DX.push_back(sc * 4.5f); DY.push_back(sc * .8f);
	ContinuousTest ctcplp{testAgents, neighbours, DX, DY, 20.f, 0};
	
	RunContinuousTest(ctcplp);
	
	//solver.computeORCAConstraints(2, 3);
	//solver.computeORCAConstraints(3, 2);
	
}

void Tester::AddTest(Test t)
{
	tests.push_back(t);
}


void Tester::RunTests()
{
	int testCount = tests.size() + contTests.size();
	int passedTest = 0;
	int testIndex = 0;
	for(Test t : tests)
	{
		testIndex++;
		
		std::cout << "Test " << testIndex << ": " << t.Vnewx << "," << t.Vnewy << "\n";
		
		if(RunTest(t))
		{
			passedTest++;
		}
	}
	
	for(ContinuousTest t : contTests)
	{
		testIndex++;
		std::cout << "Test " << testIndex << ": " ;
		if(RunContinuousTest(t))
		{
			passedTest++;
			std::cout << "success\n\n";
		}
		else
		{
			std::cout << "fail\n\n";
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


bool Tester::RunTest(Test t)
{
	solver.ClearAgents();
	std::vector<int> agentIds;
	for(int i = 0; i < t.agents.size(); i++)
	{
		agentIds.push_back(solver.AddAgent());
		solver.GetAgent(agentIds[i]) = t.agents[i];
	}
	for(auto it = t.neighbours.begin(); it != t.neighbours.end(); it++)
	{
		int i = it->first;
		int j = it->second;
		//Agent& a = solver.GetAgent(agentIds[i]);
		//Agent& b = solver.GetAgent(agentIds[j]);
		solver.SetAgentsNearby(agentIds[i], agentIds[j]);
		
	}
	solver.ComputeNewVelocities();
	
	Agent testedAgent = solver.GetAgent(agentIds[t.testedAgent]);
	
	
	std::cout << "Solver res: " << testedAgent.vx_new  << "," << testedAgent.vy_new << "\n\n";
	if(fabs(testedAgent.vx_new - t.Vnewx) < EPS && fabs(testedAgent.vy_new - t.Vnewy) < EPS)
	{
		return true;
	}
	return false;
}


void Tester::AddTest(ContinuousTest t)
{
	contTests.push_back(t);
}

bool Tester::RunContinuousTest(ContinuousTest t)
{
	solver.ClearAgents();
	
	float dt = 0.015f;
	
	
	std::vector<int> agentIds;
	for(int i = 0; i < t.agents.size(); i++)
	{
		agentIds.push_back(solver.AddAgent());
		Agent a = t.agents[i];
	
		calcPreferredVelocity(a, t.DestX[i], t.DestY[i]);
		
		a.maxAccMagnitude = a.maxAccMagnitude * dt;
		
		solver.GetAgent(agentIds[i]) = a;
	}
	for(auto it = t.neighbours.begin(); it != t.neighbours.end(); it++)
	{
		int i = it->first;
		int j = it->second;
		//Agent& a = solver.GetAgent(agentIds[i]);
		//Agent& b = solver.GetAgent(agentIds[j]);
		solver.SetAgentsNearby(agentIds[i], agentIds[j]);
	}
	
	float debugInterval = 0.1f;
	float debugTimer = 0;
	
	float timeGiven = t.timeGiven;
	while(timeGiven > 0)
	{
		timeGiven -= dt;
		debugTimer -= dt;
		
		solver.ComputeNewVelocities();
		
		if(debugTimer < 0)
		{
			debugTimer = debugInterval;
			
			char buffer[5];
			sprintf(buffer, "%.1f", timeGiven);
			std::string stime{buffer};
			SVGExporter::writeUnits("units.svg", &solver, solver.num);
			
			for(int i = 0; i < t.agents.size(); i++)
			{
				Agent& a = solver.GetAgent(i);
				std::cout << "Agent " << i << " x: " << a.x << ", y: " << a.y << " vx: " << a.vx << ", vy: " << a.vy << "\n";
				
				sprintf(buffer, "%d", i);
				std::string sid{buffer};
				
				SVGExporter::writeUnitORCAs("unitORCA" + sid + ".svg", &solver, solver.num, i);
			}
			
			std::cout << "svg done\n";
		}
		
		for(int i = 0; i < t.agents.size(); i++)
		{
			Agent& a = solver.GetAgent(i);
			
			calcPreferredVelocity(a, t.DestX[i], t.DestY[i]);
			
			a.vx = a.vx_new;
			a.vy = a.vy_new;
			a.x = a.x + a.vx * dt;
			a.y = a.y + a.vy * dt;
		}
		
	}
	
	int testAgentId = agentIds[t.testedAgent];
	Agent testedAgent = solver.GetAgent(testAgentId);
	std::cout << "x: "<< testedAgent.x << ", y: " << testedAgent.y << "\n";
	if(fabs(testedAgent.x - t.DestX[testAgentId]) < 10.f && fabs(testedAgent.y - t.DestY[testAgentId]) < 10.f)
		return true;
	else
		return false;
}


void Tester::calcPreferredVelocity(Agent& a, float DX, float DY)
{
	float toDestX = DX - a.x;
	float toDestY = DY - a.y;
	float sqrDist = toDestX * toDestX + toDestY * toDestY;
	
	float AcceptanceSquared = 400.f;
	float SlowdownSquared = 10000.f;
	float MaxVelocity = a.maxVelocityMagnitude;
	
	if (sqrDist < AcceptanceSquared)
	{
		a.vx_pref = 0.f;
		a.vy_pref = 0.f;
	}
	else
	{
		float m = 1.f / (SlowdownSquared - AcceptanceSquared);
		float b = -m * AcceptanceSquared;
		float k = sqrDist < SlowdownSquared ? sqrDist * m + b : 1.f;
		if(k < EPS)
		{
			a.vx_pref = 0.f;
			a.vy_pref = 0.f;
		}
		else
		{
			float lrec = 1.f / sqrtf(sqrDist);
			a.vx_pref = toDestX * lrec * MaxVelocity * k;
			a.vy_pref = toDestY * lrec * MaxVelocity * k;
			
		}
	}

}