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

    std::vector<Util::Point> AStarPlanner::getSuccessors(Util::Point p)
    {
        int minx = MAX(0, p.x - 1);
        int maxx = MIN(p.x + 1, gSpatialDatabase->getNumCellsX());

        int minz = MAX(0, p.z - 1);
        int maxz = MIN(p.z + 1, gSpatialDatabase->getNumCellsZ());

        std::vector<Util::Point> successors;

        for (int i = minx; i <= maxx; i++) {
            for (int j = minz; j <= maxz; j++) {
                if (i != p.x && j != p.z) {
                    int index = gSpatialDatabase->getCellIndexFromLocation(i, j);
                    if (canBeTraversed(index))
                        successors.push_back(Util::Point(i, 0, j));
                }
            }
        }
        return successors;
    }

    int findActivationNode(std::vector<AStarPlannerNode> openset)
    {
        double min_f = openset[0].f;
        int min_index = 0;
        for (int i = 1; i < openset.size(); ++i) {
            if (openset[i].f < min_f) {
                min_f = openset[i].f;
                min_index = i;
            }
        }
        return min_index;
    }

    int findNode(std::vector<AStarPlannerNode> closedset, Util::Point point)
    {
        for (size_t i = 0; i < closedset.size(); ++i)
            if (closedset[i].point == point)
                return i;
        return -1;
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

        std::vector<AStarPlannerNode> openset;
        std::vector<AStarPlannerNode> closedset;

        double f = Util::distanceBetween(start, goal);
        double g = 0;
        openset.push_back(AStarPlannerNode(start, f, g, NULL));

        while (openset.size() > 0) {
            int curr_index = findActivationNode(openset);

            if (openset[curr_index].point == goal) {
                foundPath = true;
                break;
            }

            // remove current node from open set and add to closed set
            closedset.push_back(openset[curr_index]);
            openset.erase(openset.begin() + curr_index);

            AStarPlannerNode& current = closedset.back();

            std::vector<Util::Point> successors = getSuccessors(current.point);

            // add successors to open list
            for (int i = 0; i < successors.size(); ++i) {
                int closed_index = findNode(closedset, successors[i]);
                if (closed_index != -1)
                    continue;

                double g = current.g + 1; // assuming 1 = distance from current to successors[i]
                double f = g + Util::distanceBetween(successors[i], goal);

                int open_index = findNode(openset, successors[i]);
                if (open_index == -1) {
                    // node not in openset, so add it
                    openset.push_back(AStarPlannerNode(successors[i], f, g, &current));
                } else if (g < openset[open_index].g) {
                    // node already in openset, but this is a shorter
                    // path, so update values accordingly
                    openset[open_index].f = f;
                    openset[open_index].g = g;
                    openset[open_index].parent = &current;
                }
            }
        }

        if (foundPath) {
            if (!append_to_path)
                agent_path.clear();

            int goal_index = findNode(openset, goal);

            // traverse tree to build path (in reverse order)
            std::vector<Util::Point> temp;
            AStarPlannerNode* current = &openset[goal_index];
            while (current->point != start) {
                temp.push_back(current->point);
                current = current->parent;
            }
            temp.push_back(start);

            // add planned nodes to agent path
            for (std::vector<Util::Point>::reverse_iterator iter = temp.rbegin(); iter != temp.rend(); iter++)
                agent_path.push_back(*iter);
        }

		return foundPath;
	}
}
