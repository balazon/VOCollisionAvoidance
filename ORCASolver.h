
#pragma once

#include <vector>

#define RVOTEST_API 

#define CA_MAXAGENTS (400)

#define CA_MAXNEARBY (6)

#define CA_TAU (2.f)


struct Agent
{
	float x, y;
	float vx, vy;
	float r;
	float vx_pref;
	float vy_pref;
	float vx_new;
	float vy_new;
	int nearbyAgents[CA_MAXNEARBY];

	float ORCAA[CA_MAXNEARBY];
	float ORCAB[CA_MAXNEARBY];
	float ORCAC[CA_MAXNEARBY];


	int nearbyCount;

	float maxVelocityMagnitude;
	float maxAccMagnitude;


	Agent();
	Agent(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref, float maxVelocityMagnitude, float maxAccMagnitude);

	~Agent();
};


/**
*
*/
class RVOTEST_API ORCASolver
{
public:
	ORCASolver();
	~ORCASolver();

	void ClearNeighbours(int i);
	void SetAgentsNearby(int i, int j);

	// is j a neighbour of i (true does not imply i is a neighbour of j)
	bool IsAgentNeighbour(int i, int j);


	void SetORCAConstraint(Agent& a, int j, float A, float B, float C);

	//void setAgentState(float x, float y, float vx, float vy, float r, float vx_pref, float vy_pref)


	Agent& GetAgent(int id);



	//returns id of agent
	int AddAgent();
	//returns the id of the agent who gets the removed agent's id or -1 if nobody gets it
	int RemoveAgent(int id);
	void ClearAgents();

	void SetParameters(float T);

	//new velocities which hopefully help avoid collisions
	void ComputeNewVelocities();

	void SetDebugging(bool on);

	friend class Tester;
private:

	//used for calculating the limited VO (t < T), otherwise known as tau
	float T;

	int num;
	Agent agents[CA_MAXAGENTS];


	void computeORCAConstraints(int i, int j);

};







