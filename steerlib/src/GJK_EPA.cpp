/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB)
{
    // if (A collides with B) {
    //     simplex = ...;
    //     ...
    //     return true;
    // } else {
    //     simplex = NULL
    //     return false;
    // }
    return false;
}

void SteerLib::GJK_EPA::EPA(float& penetration_depth, Util::Vector& penetration_vector, const std::vector<Util::Vector>& A, const std::vector<Util::Vector>& B, const std::vector<Util::Vector>& simplex)
{
    // Calculate penetration
    // penetration_depth = ...
    // penetration_vector = ...
}

/**
 * @brief Performs the GJK and EPA algorithms to detect
 * collisions. Note this assumes all shapes are 2D and for all points
 * (x,y,z), y=0.
 *
 * @field return_penetration_depth If there is a collision, this
 * variable is set to the penetration depth calculated by the EPA
 * algorithm
 *
 * @field return_penetration_vector If there is a collision, this
 * variable is set to the penetration vector calculated by the EPA
 * algorithm
 *
 * @field _shapeA  The first input shape
 * @field _shapeB  The second input shape
 *
 * ----------------------------------------------------
 * Algorithms:
 * Implement the GJK and EPA Algorithms in 2D since for all points (x,0,z), y=0.
 */
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    std::vector<Util::Vector> simplex;
    if (GJK(simplex, _shapeA, _shapeB)) {
        EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
        return true;
    } else {
        return_penetration_depth = 0;
        return_penetration_vector.zero();
        return false;
    }
}
