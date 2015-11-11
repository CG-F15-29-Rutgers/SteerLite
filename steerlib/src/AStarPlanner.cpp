//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

    std::vector<Util::Point> getSuccessors(Util::Point p)
    {
        std::vector<Util::Point> successors;
        return successors;
    }

    double euclideanDistance(Util::Point p1, Util::Point p2)
    {
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
    }

    std::pair<int, double> findNextNode(std::vector<AStarPlannerNode> openset, Util::Point goal)
    {
        double min_dist = euclideanDistance(openset[0].point, goal);
        int min_index = 0;
        for (int i = 1; i < openset.size(); ++i) {
            double dist = euclideanDistance(openset[i].point, goal);
            if (dist < min_dist) {
                min_dist = dist;
                min_index = i;
            }
        }
        return std::pair<int, double>(min_index, min_dist);
    }

    /**
     * Computes a path from start to goal. Returns true and populates
     * agent_path if successful (replacing existing values unless
     * append_to_path is true). Otherwise returns false.
     */
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

        bool foundPath = false;

        std::vector<AStarPlannerNode> tempPath;
        Util::Point currPoint = start;
        double f = 0;
        double g = 0;
        AStarPlannerNode* parent = NULL;
        std::vector<AStarPlannerNode> openset;
        std::vector<AStarPlannerNode> closedset;

        openset.push_back(AStarPlannerNode(start, 0, 0, NULL));

        while (openset.size() > 0) {
            std::pair<int, double> next = findNextNode(openset, goal);
            int min_index = next.first;
            double min_dist = next.second;

            if (openset[min_index].point == goal) {
                foundPath = true;
                break;
            }

            std::vector<Util::Point> successors = getSuccessors(openset[min_index].point);

            // add successors to open list
            for (int i = 0; i < successors.size(); ++i) {
                f = g + min_dist;
                g = g + euclideanDistance(successors[min_index], goal);
                openset.push_back(AStarPlannerNode(successors[i], f, g, &openset[min_index]));
            }

            // remove next node from open list
            openset.erase(openset.begin() + min_index);
        }

        if (foundPath) {
            if (!append_to_path)
                agent_path.clear();

            // add planned nodes to agent path
            // for (std::vector<AStarPlannerNode>::const_iterator iter = tempPath.begin(); iter != tempPath.end(); ++iter)
            //     agent_path.push_back(iter->point);
        }

		return foundPath;
	}
}
