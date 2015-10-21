
#pragma once


#include <string>
#include <fstream>

class ORCASolver;

class SVGExporter
{
public:
	static int writeUnits(std::string fileName, ORCASolver* solver, int num);

	static int writeUnitORCAs(std::string fileName, ORCASolver* solver, int num, int agentID, float d = 0.f);


	int writeUnits(ORCASolver* solver, int num);

	int writeUnitORCAs(ORCASolver* solver, int num, int agentID, float d);

	void writeOutLine(std::string line);

	void startSvg(int width, int height);

	void endSvg();

	void writeCircle(float u, float v, float r, std::string format = "stroke=\"black\" stroke-width=\"1\" fill=\"none\"");

	void writeVector(float x, float y, float dx, float dy);

	void writeHalfplane(float x, float y, float nx, float ny);

	void writePoint(float x, float y, std::string color);

private:

	SVGExporter(std::string fileName);

	std::ofstream out;

	void transform(float x, float y, float& resX, float& resY);


	int width, height;
	float scale;
	float Ox;
	float Oy;
};


