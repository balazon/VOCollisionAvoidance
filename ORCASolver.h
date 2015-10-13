// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#define RVOTEST_API 

#include <vector>

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
	float ux[CA_MAXNEARBY];
	float uy[CA_MAXNEARBY];
	bool uConstraintReversed[CA_MAXNEARBY];

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
	bool AreAgentsNeighbours(int i, int j);
	void SetUVector(int i, int j, float ux, float uy, bool reversed = false);

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
private:

	//used for calculating the limited VO (t < T)
	float T;

	int num;
	Agent agents[CA_MAXAGENTS];

	//void computeORCAConstraint(float Ax, float Ay, float Bx, float By, float Vax, float Vay, float Vbx, float Vby, float Ra, float Rb)

	//compute u vector: u for A agent with respect to B, and -u for agent B with respect to A
	//void computeSmallestChangeRequired

	

	void computeSmallestChangeVectors(int i, int j);

	
};







