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
#define OBSTACLE_CLEARANCE 20//1
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

		x_range_min = (x < OBSTACLE_CLEARANCE) ? 0 : (x - OBSTACLE_CLEARANCE);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX() - 1);

		z_range_min = (z < OBSTACLE_CLEARANCE) ? 0 : (z - OBSTACLE_CLEARANCE);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ() - 1);

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
/*
    bool AStarPlanner::canBeTraversed ( int id )
    {
        float traversal_cost=0;
        
    
        traversal_cost = gSpatialDatabase->getTraversalCost ( id );
        if(traversal_cost>0)
            return false;
        else
            return true;

    }*/
    
    std::vector<Util::Point> AStarPlanner::getSuccessors(const Util::Point& p)
    {
        int minx = MAX(gSpatialDatabase->getOriginX(), p.x - 1);
        int maxx = MIN(p.x + 1, gSpatialDatabase->getNumCellsX() + gSpatialDatabase->getOriginX());
        
        int minz = MAX(gSpatialDatabase->getOriginZ(), p.z - 1);
        int maxz = MIN(p.z + 1, gSpatialDatabase->getNumCellsZ() + gSpatialDatabase->getOriginZ());
        
        std::vector<Util::Point> successors;
        
        for (int i = minx; i <= maxx; i++) {
            for (int j = minz; j <= maxz; j++) {
                if (!(i == p.x && j == p.z)) {
                    int index = gSpatialDatabase->getCellIndexFromLocation(i, j);
                    if (canBeTraversed(index))
                        successors.push_back(Util::Point(i, 0, j));
                }
            }
        }
        return successors;
    }

    std::vector<Util::Point> AStarPlanner::get_partial_Successors(const Util::Point& p) //add for not consindering diagonal
    {
        int minx = MAX(gSpatialDatabase->getOriginX(), p.x - 1);
        int maxx = MIN(p.x + 1, gSpatialDatabase->getNumCellsX() + gSpatialDatabase->getOriginX());

        int minz = MAX(gSpatialDatabase->getOriginZ(), p.z - 1);
        int maxz = MIN(p.z + 1, gSpatialDatabase->getNumCellsZ() + gSpatialDatabase->getOriginZ());

        std::vector<Util::Point> successors;
        
        
        int index1 = (gSpatialDatabase->getCellIndexFromLocation(minx,p.z));
        int index2 = (gSpatialDatabase->getCellIndexFromLocation(p.x,maxz));
        int index3 = (gSpatialDatabase->getCellIndexFromLocation(maxx,p.z));
        int index4 = (gSpatialDatabase->getCellIndexFromLocation(p.x,minz));
        
        if (canBeTraversed(index1))
            successors.push_back(Util::Point(minx,0,p.z));
        if (canBeTraversed(index2))
            successors.push_back(Util::Point(p.x,0,maxz));
        if (canBeTraversed(index3))
            successors.push_back(Util::Point(maxx,0,p.z));
        if (canBeTraversed(index4))
            successors.push_back(Util::Point(p.x,0,minz));

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

    int findNode(const std::vector<AStarPlannerNode>& set, const Util::Point& point)
    {
        for (size_t i = 0; i < set.size(); ++i)
            if (set[i].point == point)
                return i;
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
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D* _gSpatialDatabase, bool append_to_path, float weight,bool diagonal)
	{
		gSpatialDatabase = _gSpatialDatabase;

        bool foundPath = false;

        //float weight = 1; // for weighted A*

        std::vector<AStarPlannerNode> openset;
        std::vector<AStarPlannerNode> closedset;

        // Set heuristic to Euclidean or Manhattan distance
        float (*heuristic)(const Util::Point& p1, const Util::Point& p2) = euclideanDistance;

        openset.push_back(AStarPlannerNode(start, weight * heuristic(start, goal), 0, -1));

        while (openset.size() > 0) {
            int curr_index = findActivationNodeLowestG(openset);

            if (openset[curr_index].point == goal) {
                foundPath = true;
                break;
            }

            // remove current node from open set and add to closed set
            closedset.push_back(openset[curr_index]);
            openset.erase(openset.begin() + curr_index);

            AStarPlannerNode& current = closedset.back();
            std::vector<Util::Point> successors;
            if(!diagonal)
           successors = get_partial_Successors(current.point);
            else
              successors = getSuccessors(current.point);
                
                
                
                
                
        

            // add successors to open list
            for (int i = 0; i < successors.size(); ++i) {
                int closed_index = findNode(closedset, successors[i]);
                if (closed_index != -1) { //in the closed set.
                    continue;
                }

                double g = current.g + 1; // part 1
                // double g = current.g + euclideanDistance(current.point, successors[i]); // part 3
                double f = g + weight * heuristic(successors[i], goal);

                int open_index = findNode(openset, successors[i]);
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

            int goal_index = findNode(openset, goal);
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
