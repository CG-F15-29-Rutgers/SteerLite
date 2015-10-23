/*!
*
* \author VaHiD AzIzI
*
*/


#ifndef GJK_EPA_H_
#define GJK_EPA_H_


#include "util/Geometry.h"


#include <vector>


namespace SteerLib
{

    class STEERLIB_API GJK_EPA
    {
        public:
			GJK_EPA();

            /*
             *  Testing:
             *
             *  To test your code, execute the following command. The
             *  output you see should be:
             *
             *  $ ./steersim -testcase polygons_test -ai collisionAI
             *
             *  loaded module collisionAI
             *  loaded module testCasePlayer
             *  Initializing...
             *  Preprocessing...
             *   Collision detected between polygon No.0 and No.1 with a penetration depth of 2 and penetration vector of (-0,0,-1)
             *  Simulation is running...
             *
             *  ----------------------------------------------------
             *
             *  Additional Notes:
             *
             *  intersection will be called for all polygons, for all
             *  unique pairs, ie. if we have 3 polygons it is called 3
             *  times:
             *
             *  one between polygon No. 1 and No. 2
             *  one between polygon No. 1 and No. 3
             *  one between polygon No. 2 and No. 3
             *
             *  For grading your results will be compared by our
             *  results on testcase polygons1
             */
            static bool intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);

        private:
            static Util::Vector support(const std::vector<Util::Vector>& shape, const Util::Vector& direction);
            static bool simplexContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction);
            static bool GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B);
            static void EPA(float& penetration_depth, Util::Vector& penetration_vector,  const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B,  std::vector<Util::Vector>& simplex);
            static void Find_Closest_edge(std::vector<Util::Vector>& simplex, Util::Vector& direction);
    }; // class GJK_EPA

} // namespace SteerLib


#endif /* GJK_EPA_H_ */
