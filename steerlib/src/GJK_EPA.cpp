#include "obstacles/GJK_EPA.h"

SteerLib::GJK_EPA::GJK_EPA()
{
}

Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& shape, const Util::Vector& direction)
{
    assert(shape.size() > 0);

    double max = dot(*shape.begin(), direction);
    Util::Vector point = *shape.begin();

    for (std::vector<Util::Vector>::const_iterator iter = shape.begin(); iter != shape.end(); ++iter) {
        double thisDot = dot(*iter, direction);
        if (thisDot > max) {
            max = thisDot;
            point = *iter;
        }
    }
    return point;
}

bool SteerLib::GJK_EPA::simplexContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction)
{
    assert(simplex.size() == 3);

    Util::Vector A = simplex[0];
    Util::Vector B = simplex[1];
    Util::Vector C = simplex[2];

    Util::Vector CB = B - C;
    Util::Vector CA = A - C;
    Util::Vector CO = -C;     // C to the origin

    Util::Vector normalToCA = cross(cross(CB, CA), CA);
    Util::Vector normalToCB = cross(cross(CA, CB), CB);

    if (dot(normalToCA, CO) > 0) {
        // origin is beyond CA
        simplex.erase(simplex.begin() + 1); // delete B
        direction = normalToCA; // new direction.
        return false;
    } else if (dot(normalToCB, CO) > 0) {
        // origin is beyond CB
        simplex.erase(simplex.begin()); // delete A
        direction = normalToCB; // new direction.
        return false;
    } else {
        // simplex contains origin
        return true;
    }
}

Util::Vector SteerLib::GJK_EPA::unit(const Util::Vector& v)
{
    return v / v.norm();
}

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB)
{
    // initial directions
    Util::Vector D0(1, 0, 0);
    Util::Vector D1(0, 0, 1);

    Util::Vector A = support(shapeA, D0) - support(shapeB, -D0);
    Util::Vector B = support(shapeA, D1) - support(shapeB, -D1);

    std::cerr << "supp(shA, D0) = " << support(shapeA, D0) << ", supp(shB, D0) = " << support(shapeB, D0) << std::endl;
    std::cerr << "A = " << A  << ", B = " << B << std::endl;
    // std::cerr << "shapeA = " << shapeA << ", shapeB = " << shapeB << std::endl;

    Util::Vector AB = B - A;
    Util::Vector AO = -A;
    simplex.clear();
    simplex.push_back(A);
    simplex.push_back(B);
    Util::Vector D = cross(cross(AB, AO), AB); // direction to 0
    while (true) {
        Util::Vector C = support(shapeA, D) - support(shapeB, -D); // new C vertex.

        std::cerr << "C = " << C << ", D = " << D << std::endl;

        if (dot(C, D) < 0)
            return false; // no collision
        simplex.push_back(C);
        if (simplexContainsOrigin(simplex, D))
            return true;
    }
}

void SteerLib::GJK_EPA::EPA(float& penetration_depth, Util::Vector& penetration_vector,  const std::vector<Util::Vector>& A,  const std::vector<Util::Vector>& B,  std::vector<Util::Vector>& simplex)
{
    while (true)
    {
        int edge_index;
        Util::Vector E0 = findClosestEdge(simplex, edge_index); // return E0 : normal vector.
        Util::Vector new_Vertex = support(A, E0) - support(B, -E0); // new point.
        // dot(O new_vertex, projection E0) ==magnitude of EO?)

        // until find minkowski
        if (fabs(E0.norm() - dot(new_Vertex, unit(E0))) < 1e-6) {
            // std::cerr << abs(E0.norm()-dot(new_Vertex,unit(E0)));
            // " Collision detected between polygon No." << i << " and No." << j <<  " with a penetration depth of " << penetration_depth << " and penetration vector of " << penetration_vector << std::endl;
            penetration_depth = E0.norm();
            penetration_vector = unit(E0);
            break;
        } else {
            // add new point to simplex and loop
            simplex.insert(simplex.begin() + edge_index, new_Vertex);
        }
    }
}

Util::Vector SteerLib::GJK_EPA::findClosestEdge(const std::vector<Util::Vector>& simplex, int& index)
{
    assert(simplex.size() > 2);

    Util::Vector A = simplex[0];
    Util::Vector B = simplex[1];
    Util::Vector C = simplex[2];

    std::vector<Util::Vector> edge;
    edge.push_back(B - A); // AB
    edge.push_back(C - B); // BC
    edge.push_back(A - C); // CA

    std::vector<float> distances;
    std::vector<Util::Vector> normal_to_edge;

    // distances.push_back(10);
    for (size_t i = 0; i < edge.size(); ++i)
    {
        normal_to_edge.push_back(cross(cross(edge[i], -simplex[i]), edge[i])); // direction.
        distances.push_back(dot(simplex[i], unit(normal_to_edge[i]))); // value
    }

    // find min distance
    index = 0;
    for (size_t i = 1; i < distances.size(); ++i)
        if (distances[index] > distances[i])
            index = i;

    return normal_to_edge[index];
}

/**
 * @brief Detect collisions using the GJK and EPA algorithms. Note
 * this assumes all shapes are 2D and y = 0 for all points.
 *
 * @field penetration_depth  If there is a collision, this variable is
 * set to the penetration depth calculated by the EPA
 * algorithm. Otherwise it's set to zero.
 *
 * @field penetration_vector  If there is a collision, this variable is
 * set to the penetration vector calculated by the EPA
 * algorithm. Otherwise it's set to the zero vector.
 *
 * @field shapeA  The first input shape
 * @field shapeB  The second input shape
 */
bool SteerLib::GJK_EPA::intersect(float& penetration_depth, Util::Vector& penetration_vector, const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB)
{
    std::vector<Util::Vector> simplex;
    if (GJK(simplex, shapeA, shapeB)) {
        EPA(penetration_depth, penetration_vector, shapeA, shapeB, simplex);
        return true;
    } else {
        penetration_depth = 0;
        penetration_vector.zero();
        return false;
    }
}
