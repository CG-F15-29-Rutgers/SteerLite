#ifndef GJK_EPA_H_
#define GJK_EPA_H_

#include <vector>

#include "util/Geometry.h"

namespace SteerLib
{
    class STEERLIB_API GJK_EPA
    {
        public:
			GJK_EPA();
            static bool intersect(float& penetration_depth, Util::Vector& penetration_vector, const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB);

        private:
            static Util::Vector support(const std::vector<Util::Vector>& shape, const Util::Vector& direction);
            static bool simplexContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction);
            static bool GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B);
            static void EPA(float& penetration_depth, Util::Vector& penetration_vector,  const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B,  std::vector<Util::Vector>& simplex);
            static void findClosestEdge(std::vector<Util::Vector>& simplex, Util::Vector& direction);
    };
}

#endif
