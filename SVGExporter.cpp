// Fill out your copyright notice in the Description page of Project Settings.

//#include "RVOTest.h"
#include "SVGExporter.h"


#include "MathUtils.h"
#include <iostream>
#include <cstdio>

#include "ORCASolver.h"

int SVGExporter::writeUnits(std::string fileName, ORCASolver* solver, int num)
{
	SVGExporter exporter{ fileName };

	if (!exporter.out.is_open())
	{
		std::cout << "ERROR: can't write " << fileName << "\n";
		return -1;
	}

	return exporter.writeUnits(solver, num);
}

int SVGExporter::writeUnitORCAs(std::string fileName, ORCASolver* solver, int num, int agentID, float d)
{
	SVGExporter exporter{ fileName };

	if (!exporter.out.is_open())
	{
		std::cout << "ERROR: can't write " << fileName << "\n";
		return -1;
	}

	return exporter.writeUnitORCAs(solver, num, agentID, d);
}

int SVGExporter::writeUnits( ORCASolver* solver, int num)
{
	startSvg(width, height);

	
	Ox = 4.5f * 42.f;
	Oy = 3.5f * 42.f;
	scale = 1.f;
	
	for (int i = 0; i < num; i++)
	{
		Agent& a = solver->GetAgent(i);
		writeCircle(a.x, a.y, a.r);
		writeVector(a.x, a.y, a.vx, a.vy);
		writePoint(a.x, a.y, "black");
	}

	endSvg();

	return 0;
}

int SVGExporter::writeUnitORCAs(ORCASolver* solver, int num, int agentID, float d)
{
	Agent& a = solver->GetAgent(agentID);
	
	

	Ox = a.x;
	Oy = a.y;
	scale = 1.f;

	
	startSvg(width, height);

	

	writePoint(a.x, a.y, "black");

	for (int i = 0; i < a.nearbyCount; i++)
	{
		float A = a.ORCAA[i];
		float B = a.ORCAB[i];
		float C = a.ORCAC[i];
		float tx, ty;
		BMU::OrthogonalProjectionOfPointOnLine(A, B, C, a.vx, a.vy, tx, ty);
		writeHalfplane(a.x + tx + A * d, a.y + ty + B * d, A, B);
	}
	writeCircle(a.x + a.vx, a.y + a.vy, a.maxAccMagnitude);
	writeCircle(a.x, a.y, a.maxVelocityMagnitude);

	writePoint(a.x + a.vx_pref, a.y + a.vy_pref, "blue");

	writePoint(a.x + a.vx_new, a.y + a.vy_new, "green");

	

	writeVector(a.x, a.y, a.vx_pref, a.vy_pref);

	writeVector(a.x, a.y, a.vx_new, a.vy_new);

	writeVector(a.x + a.vx, a.y + a.vy, a.vx_new - a.vx, a.vy_new - a.vy);

	

	endSvg();

	return 0;
}

SVGExporter::SVGExporter(std::string fileName) : out{ fileName, std::ofstream::out }
{
	scale = 40.f;
	Ox = 0.f;
	Oy = 0.f;

	width = 800;
	height = 600;
}

void SVGExporter::writeOutLine(std::string line)
{
	out << line << std::endl;
}

void SVGExporter::startSvg(int width, int height)
{
	writeOutLine("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
	writeOutLine("<svg ");
	writeOutLine("xmlns=\"http://www.w3.org/2000/svg\"");

	char buffer[50];
	sprintf(buffer, "width=\"%d\" height=\"%d\">\n", width, height);
	writeOutLine(std::string{ buffer });
}

void SVGExporter::endSvg()
{
	writeOutLine("</svg>");

}

void SVGExporter::writeCircle(float u, float v, float r, std::string format)
{
	transform(u, v, u, v);
	r *= scale;

	char buffer[150];
	sprintf(buffer, "<circle %s \n cx=\"%.5f\" cy=\"%.5f\" r=\"%.5f\"  />\n", format.c_str(), u, v, r);
	writeOutLine(std::string{ buffer });

}

void SVGExporter::writeVector(float x, float y, float dx, float dy)
{
	transform(x, y, x, y);
	dx *= scale;
	dy *= scale;
	char buffer[150];
	sprintf(buffer, "<path stroke=\"black\" stroke-width=\"1\" fill=\"none\" \n d=\"M %.4f %.4f %.4f %.4f\"  />\n", x, y, x + dx, y + dy);
	writeOutLine(std::string{ buffer });
}

void SVGExporter::writeHalfplane(float x, float y, float nx, float ny)
{

	float rotNx = -ny;
	float rotNy = nx;

	float length = 1000.f;
	float startx = x + rotNx * length;
	float starty = y + rotNy * length;
	float endx = x - rotNx * length;
	float endy = y - rotNy * length;
	writeVector(startx, starty, endx - startx, endy - starty);


	writeVector(x, y, nx * 20.f, ny * 20.f);

	std::string style{ "style=\"fill:#000000;fill-rule:evenodd;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1;fill-opacity:0.23529412\"" };

	float areawidth = 20.f;
	float areastartx = startx - nx * areawidth;
	float areastarty = starty - ny * areawidth;
	float areaendx = endx - nx * areawidth;
	float areaendy = endy - ny * areawidth;

	transform(areastartx, areastarty, areastartx, areastarty);
	transform(areaendx, areaendy, areaendx, areaendy);
	transform(startx, starty, startx, starty);
	transform(endx, endy, endx, endy);
	char buffer[300];
	sprintf(buffer, "<path %s \n d=\"M %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\"  />\n", style.c_str(), areastartx, areastarty, startx, starty, endx, endy, areaendx, areaendy);
	writeOutLine(std::string{ buffer });
}

void SVGExporter::transform(float x, float y, float& resX, float& resY)
{


	resX = x * scale - Ox + (float) width / 2.f;
	resY = y * scale - Oy + (float) height / 2.f;

}


void SVGExporter::writePoint(float x, float y, std::string color)
{
	std::string format{ "stroke=\"none\" stroke-width=\"1\" fill=\"" + color + "\"" };
	writeCircle(x, y, 3.f, format);

}