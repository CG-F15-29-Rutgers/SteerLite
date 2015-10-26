

#include "obstacles/GJK_EPA.h"
bool origin_touch_flag = false;
SteerLib::GJK_EPA::GJK_EPA()
{
   // origin_touch_flag = false;
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
    Util::Vector D0(1, 0, 0);
    Util::Vector D1(0, 0, 1);
    //Util::Vector tempA = support(shapeA, D0);
    //Util::Vector tempB= support(shapeB,D0);
    Util::Vector A = support(shapeA, D0) - support(shapeB, -D0);
    Util::Vector B = support(shapeA, D1) - support(shapeB, -D1);
    
    Util::Vector AB = B - A;
    Util::Vector AO = -A;
    simplex.clear();
    simplex.push_back(A);
    simplex.push_back(B);
    
    if (A==(0,0,0)||B==(0,0,0))
    {
        origin_touch_flag = true;
        return true;
        
    }
    
    if(dot(A,B)== -A.norm()*B.norm())
    {
        return true; // 0A and 0B are aligned.
    }
    
    Util::Vector D = cross(cross(AB, AO), AB); //direction to 0
    while (true) {
        Util::Vector C = support(shapeA, D) - support(shapeB, -D); //new C vertex.
        simplex.push_back(C);
        
        if (C==(0,0,0))
        {
            origin_touch_flag=true;
            return true;
        }
        
        
        
        if (dot(C, D) < 0)
            return false; //no collision.s
        
        //line simplex? then return true;
        //if(cross((C-A),(C-B))==(0,0,0))
        //  return true;
        bool contains_origin = simplexContainsOrigin(simplex, D);
        if (contains_origin)
            return true;
    }
}
void SteerLib::GJK_EPA::EPA(float& penetration_depth, Util::Vector& penetration_vector,  const std::vector<Util::Vector>& A,  const std::vector<Util::Vector>& B,  std::vector<Util::Vector>& simplex)
{
    
    int size_of_simplex=simplex.size();
    if (origin_touch_flag)
    {
        penetration_depth = 0;
        if(simplex[0]!=(0,0,0))
            penetration_vector = simplex[0]; //OA or
        else
            penetration_vector = simplex[1]; //OB
        
        return ;
    }
    
    ///if (size_of_simplex==2)
    
    // handle line or triangle simplex.
    
    while (true)
    {
        int edge_index;
        float min_distance_to_edge;
        
        Util::Vector E0 = findClosestEdge(A,B,simplex,edge_index,min_distance_to_edge); // return E0 : normal vector to find another vertex.
        
        E0 = -E0; //direction to the opposite way from origin.
        Util::Vector new_Vertex = support(A, E0) - support(B, -E0); // new point.
        
        float test = dot(new_Vertex,unit(E0));
        
        
        if (fabs(min_distance_to_edge - dot(new_Vertex, unit(E0))) < 1e-6) {
            
            penetration_depth =  min_distance_to_edge;
            penetration_vector = unit(-E0); // toward to origin.
            break; //loop exit.
        } else {
            
            simplex.insert(simplex.begin() + (edge_index+1), new_Vertex);
            
        }
    }
}

Util::Vector SteerLib::GJK_EPA::findClosestEdge(const std::vector<Util::Vector>& A,  const std::vector<Util::Vector>& B,const std::vector<Util::Vector>& simplex, int& index, float& distance)
{
    
    //Util::Vector test(0,0,0);
    
    
    // assert(simplex.size() > 2); we have to handel line simplex.
    std::vector<Util::Vector> edge;
    //Util::vector normal_origin_edge1;
    // Util::vector normal_origin_edge2;
    Util::Vector support_edge1;
    Util::Vector support_edge2;
    int simplex_size=simplex.size();
    
    
    
    for (size_t i =0; i< simplex_size;++i)
    {
        edge.push_back(simplex[(i+1)%simplex_size]-simplex[(i)%simplex_size]); // 01 12 23 30 , when 4 vertexes i->i+1
    }
    
    // line case just AB BA.
    
    
    
    std::vector<float> distances;
    std::vector<Util::Vector> normal_to_edge;
    
    
    for (size_t i = 0; i < edge.size(); ++i)
    {
        normal_to_edge.push_back(cross(cross(edge[i], -simplex[i]), edge[i])); // direction (ABcrossAO)cross AB
        
        if (cross(cross(edge[i], -simplex[i]), edge[i])==(0,0,0)) // edge crossing origin
        {
            Util::Vector normal_origin_edge1(-edge[i].z,0,edge[i].x);
            Util::Vector normal_origin_edge2(edge[i].z,0,-edge[i].x);
            support_edge1 = support(A,normal_origin_edge1)-support(B,-normal_origin_edge1);
            support_edge2 = support(A,normal_origin_edge2)-support(B,-normal_origin_edge2);
            
            if(dot(support_edge1,normal_origin_edge1)>0)
                if(dot(support_edge2,normal_origin_edge2)>0) //inner.
                {
                    for (size_t j= 0;j<simplex_size;++j)
                    {
                        if(support_edge1==simplex[j]) //yes then new direction.
                        {
                            distance = 0;
                            return -normal_origin_edge2 ;
                        }
                    }
                    distance =0; // no same vertex. this can be the next direction.
                    return -normal_origin_edge1;
                    
                }
                else //outer.
                {
                    distance = 0;
                    return -normal_origin_edge1;
                }
            
            
                else //normal_origin_edge2>=0
                {
                    distance =0;
                    return -normal_origin_edge2;
                }
            
            
            
        }
        distances.push_back(dot(-simplex[i], unit(normal_to_edge[i]))); // distance seems to be negative....-> so corrected.
    }
    index = 0;
    int distance_size = distances.size();
    for (size_t i = 0; i < distance_size; ++i)
    {
        if (distances[index] > distances[i])
        {
            
            index = i;
        }
    }
    
    
    distance = distances[index]; // update min distance to edge.
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
