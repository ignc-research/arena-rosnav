#include "arena_dynamic_channel/graph/accessor.h"


namespace delaunator {


/*
 * graph: the pointer of a Delaunator instance
 * face_point: the current face visited by robot
 *             each face contains three vertex indices [t, t+1, t+2]
 *             we use any of the indices as face representation
 *             the other two indices can be visited by nextHalfedge or prevHalfedge
 * neighbors: the output vector of neighbors,
 *            each of which is represented by one of the vertex indices of the adjacent triangle
 * outgoings: the corresponding vertex indices of the curren triangle
 *
**/
void neighborTriangles(const Delaunator* graph,
                       const index triangle_id,
                       std::vector<index>& neighbors,
                       std::vector<index>& outgoings)
{
    neighbors.clear();
    outgoings.clear();

    index outward = triangle_id;
    for (uint i = 0; i < 3; ++i) {
        outward = nextHalfedge(outward);
        auto opposite = graph->halfedges[outward];
        if (opposite != INVALID_INDEX){
            outgoings.push_back(outward);
            neighbors.push_back(opposite);
        } // end if
    } // end for
}



void neighborVertices(const Delaunator* graph,
                      const index cur_eid,
                      std::vector<index>& neighbors){
    neighbors.clear();

    index outward = cur_eid;
    for (uint i = 0; i < 3; ++i) {
        outward = nextHalfedge(outward);
        neighbors.push_back(outward); // vectices of the trianle
        auto income = graph->halfedges[outward];
        if (income != INVALID_INDEX) {
            // the third vertex outside the current triangle
            auto third = prevHalfedge(income);
            neighbors.push_back(third);
        } // end if
    } // end for
}




/*
 * graph: the pointer of a Delaunator instance
 * face_cent: the centroid of a trangle
**/
void calculateFaceCenters(const Delaunator* graph, std::vector<double>& center)
{
    center.reserve(graph->triangles.size());
    for(std::size_t i = 0; i < graph->triangles.size(); i+=3) {
        auto tx0 = graph->coords[2 * graph->triangles[i]];         //tx0
        auto ty0 = graph->coords[2 * graph->triangles[i] + 1];     //ty0
        auto tx1 = graph->coords[2 * graph->triangles[i + 1]];     //tx1
        auto ty1 = graph->coords[2 * graph->triangles[i + 1] + 1]; //ty1
        auto tx2 = graph->coords[2 * graph->triangles[i + 2]];     //tx2
        auto ty2 = graph->coords[2 * graph->triangles[i + 2] + 1]; //ty2
        auto ave_tx = (tx0 + tx1 + tx2) / 3.0;
        auto ave_ty = (ty0 + ty1 + ty2) / 3.0;
        center[i] = ave_tx;
        center[i+1] = ave_ty;
    }
}

/*
 ** Important **
 * 1. The start and goal of planning should participate in triangulation
 *    Once the robot runs outside the convex hull, the start point that
 *    participates in triangulation should be replaced with the current
 *    robot position.
 * 2. There is at least one triangular
 *
 *
 * graph: the pointer of a Delaunator instance
 * cx, cy: the coordinite of test point
 * output: the index of triangular vertex
 * distance: the squared distance to the nearest obstacle
 *
 * return fasle if (px, py) lies in a region controlled by pedestrians
 *
 * return true if successfully locate the face. Invalid means out side the convex hull
 * the output the traingle vertex index of the nearest vertex, i.e., the eid
 *
 *
**/
bool locateCurrentFace(const Delaunator* graph,
                       const double px, const double py,
                       index& output, double& distance,
                       double SAFE_DIST, size_t PED_START)
{
    // at least a triangle is found
    if (graph->triangles.size() == 0){
        throw std::runtime_error("No triangle found!");
    }

    // find the closest point to the current point
    auto n = graph->coords.size() >> 1;
    double min_dist_sq = DBL_MAX;
    rank i0 = INVALID_INDEX;
    for (uint i = 0; i < n; i++) {
        // squared distance
        double d = dist(px, py, graph->coords[2*i], graph->coords[2*i+1]);
        if (d < min_dist_sq) {
            i0 = i;
            min_dist_sq = d;
        }
    }

    // record the minimum Euclidean distance
    distance = sqrt(min_dist_sq);

    // overlap with a certain vertex (No. 0-5 not inculded)
    if (distance < SAFE_DIST && i0 >= PED_START){
        printf("The current point is in dangerous zone!\n");
        output = INVALID_INDEX;
        return false; // collision happens
    }

    // locate the trangle index of the closest point
    index start = INVALID_INDEX;
    for (uint i = 0; i < graph->triangles.size(); ++i) {
        if(graph->triangles[i] == i0){
            start = i;
            break;
        }
    }

    // edges around a graph node
    // ->incoming -->outcoming -->third -
    auto outgoing = start;
    auto incoming_pair = graph->halfedges[outgoing];
    do{
        auto third = nextHalfedge(outgoing);
        auto incoming = nextHalfedge(third);
        // test if inside the triangle
        auto ax = graph->coords[2 * graph->triangles[incoming] + 0];
        auto ay = graph->coords[2 * graph->triangles[incoming] + 1];
        auto bx = graph->coords[2 * graph->triangles[outgoing] + 0];
        auto by = graph->coords[2 * graph->triangles[outgoing] + 1];
        auto cx = graph->coords[2 * graph->triangles[third] + 0];
        auto cy = graph->coords[2 * graph->triangles[third] + 1];
        if (isInsideTriangle(px, py, ax, ay, bx, by, cx, cy)){
            output = outgoing;
            return true;
        }
        // next triangle
        outgoing = graph->halfedges[incoming];
    } while(outgoing != INVALID_INDEX && outgoing != start);


    // the start point lies in convex hull and the iteration starts from middle
    if (outgoing == INVALID_INDEX && incoming_pair != INVALID_INDEX){
        auto incoming = incoming_pair;
        do{
            outgoing = nextHalfedge(incoming);
            auto third = nextHalfedge(outgoing);

            // fetch coordinates and test if inside the triangle
            auto ax = graph->coords[2 * graph->triangles[incoming]];
            auto ay = graph->coords[2 * graph->triangles[incoming] + 1];
            auto bx = graph->coords[2 * graph->triangles[outgoing]];
            auto by = graph->coords[2 * graph->triangles[outgoing] + 1];
            auto cx = graph->coords[2 * graph->triangles[third]];
            auto cy = graph->coords[2 * graph->triangles[third] + 1];

            if (isInsideTriangle(px, py, ax, ay, bx, by, cx, cy)){
                output = outgoing;
                return true;
            }
            // next triangle
            incoming = graph->halfedges[outgoing];
        } while(incoming != INVALID_INDEX);
    }

    // not in any trianular, which means outside the convex hull
    output = INVALID_INDEX;
    return true;
}

size_t locateCurrentFace(const Delaunator* graph, const double px, const double py) {
   
    // at least a triangle is found
    if (graph->triangles.size() == 0) {
        throw std::runtime_error("No triangle found!");
    }
    
    for (size_t i = 0; i < graph->triangles.size(); i+=3) {
        auto ax = graph->coords[2 * graph->triangles[i]];
        auto ay = graph->coords[2 * graph->triangles[i] + 1];

        auto bx = graph->coords[2 * graph->triangles[i+1]];
        auto by = graph->coords[2 * graph->triangles[i+1] + 1];

        auto cx = graph->coords[2 * graph->triangles[i+2]];
        auto cy = graph->coords[2 * graph->triangles[i+2] + 1];
        
        if (isInsideTriangle(px, py, ax, ay, bx, by, cx, cy)) {
           
            return i;
        }
    }
    
    return INVALID_INDEX;
}


}