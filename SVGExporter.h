#pragma once

#include "ORCASolver.h"
#include <string>
#include <fstream>

class SVGExporter
{
public:
	static int writeUnits(std::string fileName, ORCASolver* solver, int num);
	
	static int writeUnitORCAs(std::string fileName, ORCASolver* solver, int num, int agentID);
	
	
	void writeOutLine(std::string line);
	
	void startSvg(int width, int height);
	
	void endSvg();
	
	void writeCircle(float u, float v, float r, std::string format = "stroke=\"black\" stroke-width=\"1\" fill=\"none\"");
	
	void writeVector(float x, float y, float dx, float dy);
	
	void writeHalfplane(float x, float y, float nx, float ny);
	
	void writePoint(float x, float y);
	
private:

	SVGExporter(std::string fileName);
	
	std::ofstream out;
	
	void transform(float x, float y, float& resX, float& resY);
	
	
	float scale;
	float Ox;
	float Oy;
};


