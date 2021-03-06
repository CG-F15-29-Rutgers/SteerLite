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
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id, int obstacleClearance )
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = (x < obstacleClearance) ? 0 : (x - obstacleClearance);
		x_range_max = MIN(x+obstacleClearance, gSpatialDatabase->getNumCellsX() - 1);

		z_range_min = (z < obstacleClearance) ? 0 : (z - obstacleClearance);
		z_range_max = MIN(z+obstacleClearance, gSpatialDatabase->getNumCellsZ() - 1);

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

    std::vector<Util::Point> AStarPlanner::getSuccessors(const Util::Point& p, bool diagonal, int obstacleClearance)
    {
        int startIndex = gSpatialDatabase->getCellIndexFromLocation(p);
        unsigned int x, z;
        gSpatialDatabase->getGridCoordinatesFromIndex(startIndex, x, z);

        int minx = (x == 0) ? 0 : (x - 1);
        int maxx = MIN(x + 1, gSpatialDatabase->getNumCellsX() - 1);

        int minz = (z == 0) ? 0 : (z - 1);
        int maxz = MIN(z + 1, gSpatialDatabase->getNumCellsZ() - 1);

        std::vector<Util::Point> successors;

        for (int i = minx; i <= maxx; i++) {
            for (int j = minz; j <= maxz; j++) {
                if (!diagonal && i != x && j != z)
                    continue;
                if (!(i == x && j == z)) {
                    int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
                    if (canBeTraversed(index, obstacleClearance)) {
                        Util::Point p;
                        gSpatialDatabase->getLocationFromIndex(index, p);
                        successors.push_back(p);
                    }
                }
            }
        }
        return successors;
    }

    std::vector<int> getMinIndices(const std::vector<AStarPlannerNode>& openset)
    {
        assert(openset.size() > 0);

        double min_f = openset[0].f;
        int min_index = 0;
        std::vector<int> min_indices;
        min_indices.push_back(0);
        for (int i = 1; i < openset.size(); ++i) {
            if (openset[i].f < min_f) {
                min_f = openset[i].f;
                min_indices.clear();
                min_indices.push_back(i);
            } else if (openset[i].f == min_f) {
                min_indices.push_back(i);
            }
        }

        return min_indices;
    }

    /**
     * Finds the next node from the open set to activate, by finding
     * the node with the lowest f value. In the case of multiple such
     * nodes, it returns the first in the open set.
     */
    int findActivationNodeFirst(const std::vector<AStarPlannerNode>& openset)
    {
        std::vector<int> min_indices = getMinIndices(openset);
        return min_indices[0];
    }

    /**
     * Finds the next node from the open set to activate, by finding
     * the node with the lowest f value. In the case of multiple such
     * nodes, it chooses the one with the lowest g value.
     */
    int findActivationNodeLowestG(const std::vector<AStarPlannerNode>& openset)
    {
        std::vector<int> min_indices = getMinIndices(openset);

        double min_g = openset[min_indices[0]].g;
        double min_g_index = min_indices[0];

        for (int i = 1; i < min_indices.size(); ++i) {
            if (openset[min_indices[i]].g < min_g) {
                min_g = openset[min_indices[i]].g;
                min_g_index = min_indices[i];
            }
        }

        return min_g_index;
    }

    /**
     * Finds the next node from the open set to activate, by finding
     * the node with the lowest f value. In the case of multiple such
     * nodes, it chooses the one with the highest g value.
     */
    int findActivationNodeHighestG(const std::vector<AStarPlannerNode>& openset)
    {
        std::vector<int> min_indices = getMinIndices(openset);

        double max_g = openset[min_indices[0]].g;
        double max_g_index = min_indices[0];

        for (int i = 1; i < min_indices.size(); ++i) {
            if (openset[min_indices[i]].g > max_g) {
                max_g = openset[min_indices[i]].g;
                max_g_index = min_indices[i];
            }
        }

        return max_g_index;
    }

    int findNode(const std::vector<AStarPlannerNode>& set, const Util::Point& point, SteerLib::GridDatabase2D* _gSpatialDatabase)
    {
        int point_index = _gSpatialDatabase->getCellIndexFromLocation(point);
        for (size_t i = 0; i < set.size(); ++i) {
            int curr_index = _gSpatialDatabase->getCellIndexFromLocation(set[i].point);
            if (curr_index == point_index)
                return i;
        }
        return -1;
    }

    float euclideanDistance(const Util::Point& p1, const Util::Point& p2)
    {
        return Util::distanceBetween(p1, p2);
    }

    float manhattanDistance(const Util::Point& p1, const Util::Point& p2)
    {
        return (abs(p2.x - p1.x) + abs(p2.y - p1.y) + abs(p2.z - p1.z));
    }

    /**
     * Computes a path from start to goal. Returns true and populates
     * agent_path if successful (replacing existing values unless
     * append_to_path is true). Otherwise returns false.
     */
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D* _gSpatialDatabase, bool append_to_path, float weight, bool diagonal, int obstacleClearance)
	{
		gSpatialDatabase = _gSpatialDatabase;

        bool foundPath = false;

        std::vector<AStarPlannerNode> openset;
        std::vector<AStarPlannerNode> closedset;

        // Set heuristic to Euclidean or Manhattan distance
        float (*heuristic)(const Util::Point& p1, const Util::Point& p2) = euclideanDistance;

        openset.push_back(AStarPlannerNode(start, weight * heuristic(start, goal), 0, -1));

        while (openset.size() > 0) {
            int curr_index = findActivationNodeLowestG(openset);

            int goal_grid_index = _gSpatialDatabase->getCellIndexFromLocation(goal);
            int curr_grid_index = _gSpatialDatabase->getCellIndexFromLocation(openset[curr_index].point);

            if (goal_grid_index == curr_grid_index) {
                // std::cout << "Found path" << std::endl;
                foundPath = true;
                break;
            }

            // remove current node from open set and add to closed set
            closedset.push_back(openset[curr_index]);
            openset.erase(openset.begin() + curr_index);

            AStarPlannerNode& current = closedset.back();
            std::vector<Util::Point> successors = getSuccessors(current.point, diagonal, obstacleClearance);

            // add successors to open list
            for (int i = 0; i < successors.size(); ++i) {
                int closed_index = findNode(closedset, successors[i], _gSpatialDatabase);
                if (closed_index != -1) { // in the closed set
                    continue;
                }

                double g = current.g + 1; // part 1
                // double g = current.g + euclideanDistance(current.point, successors[i]); // part 3
                double f = g + weight * heuristic(successors[i], goal);

                int open_index = findNode(openset, successors[i], _gSpatialDatabase);
                if (open_index == -1) {
                    // node not in openset, so add it
                    openset.push_back(AStarPlannerNode(successors[i], f, g, closedset.size() - 1));
                } else if (g < openset[open_index].g) {
                    // node already in openset, but this is a shorter
                    // path, so update values accordingly
                    openset[open_index].f = f;
                    openset[open_index].g = g;
                    openset[open_index].parent_index = closedset.size() - 1;
                }
            }
        }

        if (foundPath) {
            if (!append_to_path)
                agent_path.clear();

            int goal_index = findNode(openset, goal, _gSpatialDatabase);
            assert(goal_index != -1);

            // traverse tree to build path (in reverse order)
            std::vector<Util::Point> temp;
            const AStarPlannerNode* current = &openset[goal_index];
            while (current->point != start) {
                temp.push_back(current->point);
                current = &closedset[current->parent_index];
            }
            temp.push_back(start);

            // add planned nodes to agent path
            for (std::vector<Util::Point>::reverse_iterator iter = temp.rbegin(); iter != temp.rend(); iter++) {
                agent_path.push_back(*iter);
            }
        }

		return foundPath;
	}
}
