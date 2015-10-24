/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& shape, const Util::Vector& direction)
{
    double max = 0.0;
    Util::Vector point;

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
        direction = normalToCA;
        return false;
    } else if (dot(normalToCB, CO) > 0) {
        // origin is beyond CB
        simplex.erase(simplex.begin()); // delete A
        direction = normalToCB;
        return false;
    } else {
        // simplex contains origin
        return true;
    }
}

bool SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& simplex, const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB)
{
    Util::Vector D0(1, 0, 0);
    Util::Vector D1(0, 0, 1);
    Util::Vector A = support(shapeA, D0) - support(shapeB, -D0);
    Util::Vector B = support(shapeA, D1) - support(shapeB, -D1);
    Util::Vector AB = B - A;
    Util::Vector AO = -A;
    simplex.clear();
    simplex.push_back(A);
    simplex.push_back(B);
    Util::Vector D = cross(cross(AB, AO), AB);
    while (true) {
        Util::Vector C = support(shapeA, D) - support(shapeB, -D);
        if (dot(C, D) < 0)
            return false;
        simplex.push_back(C);
        bool contains_origin = simplexContainsOrigin(simplex, D);
        if (contains_origin)
            return true;
    }
}


void SteerLib::GJK_EPA::EPA(float& penetration_depth, Util::Vector& penetration_vector,  const std::vector<Util::Vector>& A,  const std::vector<Util::Vector>& B,  std::vector<Util::Vector>& simplex)
{
    // Calculate penetration
    // penetration_depth = ...
    // penetration_vector = ...
    Util::Vector E0(0,0,0);
    
    while(true)
    {
        
        SteerLib::GJK_EPA::Find_Closest_edge(simplex,E0); //return E0 : normal vector.
        Util::Vector new_Vertex = support(A,E0)-support(B,-E0); //new point.
        //dot(O new_vertex,projection E0) ==magnitude of EO?)
        
        
        
        if (abs(E0.norm()-dot(new_Vertex,E0/E0.norm()))<0.00001) // until find minkowski
        {
            //std::cerr<<abs(E0.norm()-dot(new_Vertex,E0/E0.norm()));
            //" Collision detected between polygon No."<<i<<" and No."<<j<< " with a penetration depth of "<< penetration_depth << " and penetration vector of "<<penetration_vector<<std::endl;
            penetration_depth = E0.norm();
            penetration_vector = E0/E0.norm();
            break;
        }
        
        else
            //add simplex.
            simplex.push_back(new_Vertex); // and loop.
        
    }
    
}

void SteerLib::GJK_EPA::Find_Closest_edge(std::vector<Util::Vector>& simplex, Util::Vector& direction)
{
    Util::Vector A = simplex[0];
    Util::Vector B = simplex[1];
    Util::Vector C = simplex[2];
    
    std::vector<Util::Vector> edge;
    edge.push_back(B-A); //AB
    edge.push_back(C-B); //BC
    edge.push_back(A-C); //CA
    
    std::vector<float> min_value;
    std::vector<Util::Vector> normal_to_edge;
    
   // std::vector<Util::Vector> normal_to edge;
    min_value.push_back(10);
    for (int i=0;i<sizeof(normal_to_edge);++i)
    {
        normal_to_edge.push_back(cross(cross(edge[i],-simplex[i]),edge[i])); //direction.
        min_value.push_back(dot(simplex[i],normal_to_edge[i]/normal_to_edge[i].norm())); //value
        
    }
    
    //find min value;
    
    int min_index=0;
    
    for (int i=0;i<sizeof(min_value);++i)
    {
        if(min_value[min_index]>min_value[i])
            min_index=i;
        
    }
    direction = normal_to_edge[min_index];
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
