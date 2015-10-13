#include "SVGExporter.h"

#include <iostream>
#include <cstdio>

int SVGExporter::writeUnits(std::string fileName, Agent* agents, int num)
{
	SVGExporter exporter{fileName};
	
	if(!exporter.out.is_open())
	{
		std::cout << "ERROR: can't write " << fileName << "\n";
		return -1;
	}
	
	exporter.startSvg(800, 600);
	for(int i = 0; i < num; i++)
	{
		Agent& a = agents[i];
		exporter.writeCircle(a.x, a.y, a.r);
	}
	
	exporter.endSvg();
	
	return 0;
}
	
int SVGExporter::writeUnitORCAs(std::string fileName, Agent* agents, int num, int agentID)
{
	SVGExporter exporter{fileName};
	if(!exporter.out.is_open())
	{
		std::cout << "ERROR: can't write " << fileName << "\n";
		return -1;
	}
	
	return 0;
}

SVGExporter::SVGExporter(std::string fileName) : out{fileName}
{
	
}

void SVGExporter::writeOutLine(std::string line)
{
	out << line << std::endl;
}

void SVGExporter::startSvg(int width, int height)
{
	writeOutLine("(<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>)");
	writeOutLine("(<svg )");
	writeOutLine("(xmlns=\"http://www.w3.org/2000/svg\")");
	
	char buffer [50];
	sprintf (buffer, "width=\"%d\" height=\"%d\">", width, height);
	writeOutLine(std::string{buffer});
}
	
void SVGExporter::endSvg()
{
	writeOutLine("</svg>");

}

void SVGExporter::writeCircle(float u, float v, float r)
{
	char buffer [150];
	sprintf (buffer, "<circle stroke=\"black\" stroke-width=\"1\" fill=\"none\" cx=\"%.5f\" cy=\"%.5f\" r=\"%.5f\"  />", u, v, r);
	writeOutLine(std::string{buffer});

}