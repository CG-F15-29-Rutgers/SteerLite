//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window) //every frame called.
{
#ifdef ENABLE_GUI
    window=1;
    int size_inputPoints = controlPoints.size();
    int final_time = controlPoints[size_inputPoints-1].time;//last time.
    unsigned int nextPoint;

    Point current_point;
    Point past_point;
    for (int t = 0; t<=final_time;t=t+window)
    {
        past_point=current_point;


        bool find_time = findTimeInterval(nextPoint,t);

        if (type == hermiteCurve)
            current_point=useHermiteCurve(nextPoint,t); //compute current position at t.
        else if (type==catmullCurve)
            current_point=useCatmullCurve(nextPoint,t);


        DrawLib::drawLine(past_point,current_point,curveColor,2.0f);
    }
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
    // bubble sort
    for (int i = 0; i < controlPoints.size(); ++i)
    {
        for (int j = 0; j < controlPoints.size() - (i+1); ++j)
        {
            if (controlPoints[j].time > controlPoints[j+1].time)
            {
                float temp_swap = controlPoints[j].time;
                controlPoints[j].time = controlPoints[j+1].time;
                controlPoints[j+1].time = temp_swap;
            }
        }
    }
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
// 2015.09.27 Diana Kim
bool Curve::checkRobust()
{
    return controlPoints.size() >= 2;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
    int size_inputPoints = controlPoints.size();
    for (int i=0; i<size_inputPoints; ++i)
    {
        if(controlPoints[i].time<=time && controlPoints[i+1].time>time)
        {
            nextPoint = i+1; //next control points.
            return true;
        }
    }

    // if you still not find.
    if (time>=controlPoints[size_inputPoints-1].time) //if there is no next value.
        return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

    std::vector<float> blending_fnc;


	const CurvePoint start_point = controlPoints[nextPoint-1];
	const CurvePoint end_point=controlPoints[nextPoint];
	//const CurvePoint& end_point=controlPoints[nextPoint]

    // 2015.09.27 //Diana Kim
    intervalTime = end_point.time-start_point.time;
    normalTime =(time-start_point.time)/intervalTime;

	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Hermite curve

	// Return result
	//2. compute blending function value. //by hermite.
	blending_fnc.push_back(2*pow(normalTime,3)-3*pow(normalTime,2)+1);
	blending_fnc.push_back(-2*pow(normalTime,3)+3*pow(normalTime,2));
	blending_fnc.push_back(pow(normalTime,3)-2*pow(normalTime,2)+normalTime);
	blending_fnc.push_back(pow(normalTime,3)-pow(normalTime,2));

	//2. update position.
    newPosition= start_point.position*blending_fnc[0]+end_point.position*blending_fnc[1]+start_point.tangent*intervalTime*blending_fnc[2]+end_point.tangent*intervalTime*blending_fnc[3];

    return newPosition;
}

static Vector catmull_m(const CurvePoint& p1, const CurvePoint& p2)
{
    return (p2.position - p1.position) / (p2.time - p1.time);
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

    CurvePoint p_0 = controlPoints[nextPoint - 1];
    CurvePoint p_1 = controlPoints[nextPoint + 0];

    // Calculate time interval, and normal time required for later curve calculations
    float intervalTime = p_1.time - p_0.time;
    float normalTime = (time - p_0.time) / intervalTime;

	// Calculate position at t = time on Catmull-Rom curve
    Vector m_0;
    Vector m_1;
    if (nextPoint == 1) {
        // Estimate tangent for point 0 by (p1 - p0) / (t1 - t0)
        CurvePoint p_next = controlPoints[nextPoint + 1];
        m_0 = catmull_m(p_0, p_1);
        m_1 = catmull_m(p_0, p_next);
    } else if (nextPoint == controlPoints.size() - 1) {
        // Estimate tangent for point n by (p_n - p_{n-1}) / (t_n - t_{n-1})
        CurvePoint p_prev = controlPoints[nextPoint - 2];
        m_0 = catmull_m(p_prev, p_1);
        m_1 = catmull_m(p_0, p_1);
    } else {
        CurvePoint p_prev = controlPoints[nextPoint - 2];
        CurvePoint p_next = controlPoints[nextPoint + 1];
        m_0 = catmull_m(p_prev, p_1);
        m_1 = catmull_m(p_0, p_next);
    }

    // Blending functions
    float h00 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
    float h01 = -2 * pow(normalTime, 3) + 3 * pow(normalTime, 2);
    float h10 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
    float h11 = pow(normalTime, 3) - pow(normalTime, 2);

    newPosition = h00 * p_0.position
                + h01 * p_1.position
                + intervalTime * h10 * m_0
                + intervalTime * h11 * m_1;

	return newPosition;
}
