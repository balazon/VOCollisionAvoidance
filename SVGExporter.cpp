#include "SVGExporter.h"

#include "MathUtils.h"
#include <iostream>
#include <cstdio>

int SVGExporter::writeUnits(std::string fileName, ORCASolver* solver, int num)
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
		Agent& a = solver->GetAgent(i);
		exporter.writeCircle(a.x, a.y, a.r);
		exporter.writeVector(a.x, a.y, a.vx, a.vy);
	}
	
	exporter.endSvg();
	
	return 0;
}
	
int SVGExporter::writeUnitORCAs(std::string fileName, ORCASolver* solver, int num, int agentID)
{
	SVGExporter exporter{fileName};
	if(!exporter.out.is_open())
	{
		std::cout << "ERROR: can't write " << fileName << "\n";
		return -1;
	}
	
	exporter.startSvg(800, 600);
	
	Agent& a = solver->GetAgent(agentID);
	for(int i = 0; i < a.nearbyCount; i++)
	{
		bool rev = a.uConstraintReversed[i];
		float ux = a.ux[i];
		float uy = a.uy[i];
		if(fabs(ux) < EPS && fabs(uy) < EPS)
		{
			continue;
		}
		float nx = rev ? ux : -ux;
		float ny = rev ? uy : -uy;
		float lrec = 1.f / sqrtf(nx * nx + ny * ny);
		nx *= lrec;
		ny *= lrec;
		
		exporter.writeHalfplane(a.x + a.vx + ux * .5f, a.y + a.vy + uy * .5f, nx, ny);
	}
	exporter.writeCircle(a.x + a.vx, a.y + a.vy, a.maxAccMagnitude);
	exporter.writeCircle(a.x, a.y, a.maxVelocityMagnitude);
	
	exporter.writeVector(a.x, a.y, a.vx_pref, a.vy_pref);
	
	exporter.endSvg();
	
	return 0;
}

SVGExporter::SVGExporter(std::string fileName) : out{fileName}
{
	scale = 40.f;
	Ox = 0.f;
	Oy = 0.f;
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
	
	char buffer [50];
	sprintf (buffer, "width=\"%d\" height=\"%d\">\n", width, height);
	writeOutLine(std::string{buffer});
}
	
void SVGExporter::endSvg()
{
	writeOutLine("</svg>");

}

void SVGExporter::writeCircle(float u, float v, float r)
{
	transform(u, v, u, v);
	r *= scale;
	char buffer [150];
	sprintf (buffer, "<circle stroke=\"black\" stroke-width=\"1\" fill=\"none\" \n cx=\"%.5f\" cy=\"%.5f\" r=\"%.5f\"  />\n", u, v, r);
	writeOutLine(std::string{buffer});

}

void SVGExporter::writeVector(float x, float y, float dx, float dy)
{
	transform(x, y, x, y);
	dx *= scale;
	dy *= scale;
	char buffer [150];
	sprintf(buffer, "<path stroke=\"black\" stroke-width=\"1\" fill=\"none\" \n d=\"M %.4f %.4f %.4f %.4f\"  />\n", x, y, x+dx, y+dy);
	writeOutLine(std::string{buffer});
}

void SVGExporter::writeHalfplane(float x, float y, float nx, float ny)
{
	
	float rotNx = -ny;
	float rotNy = nx;
	
	float length = 20.f;
	float startx = x + rotNx * length;
	float starty = x + rotNy * length;
	float endx = x - rotNx * length;
	float endy = x - rotNy * length;
	writeVector(startx, starty, endx - startx, endy - starty);
	
	std::string style{"style=\"fill:#000000;fill-rule:evenodd;stroke:none;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1;fill-opacity:0.23529412\""};
	
	float areawidth = 1.f;
	float areastartx = startx - nx * areawidth;
	float areastarty = starty - ny * areawidth;
	float areaendx = endx - nx * areawidth;
	float areaendy = endy - ny * areawidth;
	
	transform(areastartx, areastarty, areastartx, areastarty);
	transform(areaendx, areaendy, areaendx, areaendy);
	transform(startx, starty, startx, starty);
	transform(endx, endy, endx, endy);
	char buffer [300];
	sprintf(buffer, "<path %s \n d=\"M %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\"  />\n", style.c_str(), areastartx, areastarty, startx, starty, endx, endy, areaendx, areaendy);
	writeOutLine(std::string{buffer});
}

void SVGExporter::transform(float x, float y, float& resX, float& resY)
{
	
	
	resX = x * scale + Ox;
	resY = y * scale + Oy;

}