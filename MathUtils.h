#pragma once

bool IntersectLines(float A, float B, float C, float D, float E, float F, float& resX, float& resY);

void OrthogonalProjectionOfPointOnLine(float A, float B, float C, float tx, float ty, float& resX, float& resY);

bool QuadraticEquation(float a, float b, float c, float& x1, float& x2);

bool IntersectLineCircle(float A, float B, float C, float u, float v, float r, float& x1, float& y1, float& x2, float& y2);

bool IntersectCircleCircle(float u1, float v1, float r1, float u2, float v2, float r2, float& x1, float& y1, float& x2, float& y2);

void OrthogonalProjectionOfPointOnCircle(float u, float v, float r, float tx, float ty, float& resX, float& resY);

