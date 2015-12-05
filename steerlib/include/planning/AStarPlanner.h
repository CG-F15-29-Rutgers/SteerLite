//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{
	/**
     * @function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
     *
     * @attributes
     *
     * f            : the f value of the node
     * g            : the cost from the start, for the node
     * point        : the point in (x,0,z) space that corresponds to the current node
     * parent_index : the index to the parent AStarPlannerNode in the closed set, so that retracing the path is possible.
     *
     * @operators
     *
     * The greater than, less than and equals operator have been overloaded.
     */
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			size_t parent_index;

			AStarPlannerNode(Util::Point _point, double _f, double _g, size_t _parent_index)
			{
				f = _f;
				point = _point;
				g = _g;
				parent_index = _parent_index;
			}

			bool operator<(const AStarPlannerNode& other) const
		    {
		        return this->f < other.f;
		    }

		    bool operator>(const AStarPlannerNode& other) const
		    {
		        return this->f > other.f;
		    }

		    bool operator==(const AStarPlannerNode& other) const
		    {
		        return ((this->point.x == other.point.x) &&
                        (this->point.z == other.point.z));
		    }
	};

    // NOTE: There are four indices that need to be distinguished
    //
    // -- Util::Points in 3D space(with Y=0)
    // -- (double X, double Z) Points with the X and Z coordinates of the actual points
    // -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D.
    //    The Grid database can start from any physical point (say -100,-100). So X_GRID and X need not match.
    // -- int GridIndex is the index of the GRID data structure. This is an unique id mapping to every cell.

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();

			/**
             * @function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
             * The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
             * and checks cells in bounding box area
             * [[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
             * [Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
             * This function also contains the griddatabase call that gets traversal costs.
             */
			bool canBeTraversed ( int id, int obstacleClearance );

			/**
             * @function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
             */
			Util::Point getPointFromGridIndex(int id);

			/**
             * @function computePath
             *
             * This function executes an A* query
             *
             * @parameters
             *
             * agent_path        : The solution path that is populated by the A* search
             * start             : The start point
             * goal              : The goal point
             * _gSpatialDatabase : The pointer to the GridDatabase2D from the agent
             * append_to_path    : An optional argument to append to agent_path instead of overwriting it.
             */

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D* _gSpatialDatabase, bool append_to_path = false, float weight=1, bool diagonal=true, int obstacleClearance = 0);

        private:
			SteerLib::GridDatabase2D* gSpatialDatabase;

            std::vector<Util::Point> getSuccessors(const Util::Point& p, bool diagonal, int obstacleClearance);
	};


}


#endif
