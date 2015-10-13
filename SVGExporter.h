#pragma once

#include "ORCASolver.h"
#include <string>
#include <fstream>

class SVGExporter
{
public:
	static int writeUnits(std::string fileName, Agent* agents, int num);
	
	static int writeUnitORCAs(std::string fileName, Agent* agents, int num, int agentID);
	
	
	void writeOutLine(std::string line);
	
	void startSvg(int width, int height);
	
	void endSvg();
	
	void writeCircle(float u, float v, float r);
	
private:

	SVGExporter(std::string fileName);
	
	std::ofstream out;
	
	
};


