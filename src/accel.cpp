/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>
#include <Eigen/Geometry>
#include <chrono>

NORI_NAMESPACE_BEGIN


// multi meshes
void Accel::addMesh(Mesh* mesh)
{
    if (m_mesh_nums >= MAX_MESH_NUMS)
        throw NoriException("Accel: only %d mesh is supported!", MAX_MESH_NUMS);
    m_meshes[m_mesh_nums] = mesh;
    m_mesh_nums++;
    m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::build()
{
    auto start = std::chrono::high_resolution_clock::now();

    int totalTriangleCnt = 0;
    for (int meshIdx = 0; meshIdx < m_mesh_nums; meshIdx++)
        totalTriangleCnt += m_meshes[meshIdx]->getTriangleCount();
    
    std::vector<uint32_t> triangleIndices(totalTriangleCnt); // 所有三角形的索引顺序存储在vector中 [5, 2, 1, 0, 3, 4,     2, 3, 1, 0,   ....]
    std::vector<uint32_t> meshIndices(totalTriangleCnt); // 所有三角形对应的mesh索引顺序存储在vector中 [0, 0, 0, 0, 0, 0,     1, 1, 1, 1,   ....]

    uint32_t offset = 0;
    for (int meshIdx = 0; meshIdx < m_mesh_nums; meshIdx++)
    {
        uint32_t curMeshTriangleCnt = m_meshes[meshIdx]->getTriangleCount();
        for (int i = 0; i < curMeshTriangleCnt; i++)
        {
            triangleIndices[offset + i] = i;
            meshIndices[offset + i] = meshIdx;
        }
        offset += curMeshTriangleCnt;
    }

    m_root = buildRecursive(m_bbox, triangleIndices, meshIndices, 0);
    printf("Octree build time: %ldms \n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
    printf("Num nodes: %d \n", m_num_nodes);
    printf("Num leaf nodes: %d \n", m_num_leaf_nodes);
    printf("Total number of saved triangles: %d \n", m_num_triangles_saved);
    printf("Avg triangles per node: %f \n", (float)m_num_triangles_saved / (float)m_num_nodes);
    printf("Recursion depth: %d \n", m_recursion_depth);
}



Accel::Node* Accel::buildRecursive(BoundingBox3f& bbox, std::vector<uint32_t>& triangleIndices, std::vector<uint32_t>& meshIndices, uint32_t recursiveDepth)
{
    m_num_nodes++;
    int triangleNum = triangleIndices.size();
    if (triangleNum == 0)
    {
        return nullptr;
    }

    //leaf node
    if (triangleNum < 10 || recursiveDepth > 15)
    {
        Node* node = new Node();
        node->triangleNum = triangleNum;
        node->triangleIndices = new uint32_t[triangleNum];
        node->meshIndices = new uint32_t[triangleNum];
        for (int i = 0; i < triangleNum; i++)
        {
            node->triangleIndices[i] = triangleIndices[i];
            node->meshIndices[i] = meshIndices[i];
        }
        node->bbox = BoundingBox3f(bbox);
        m_num_leaf_nodes++;
        m_num_triangles_saved += triangleNum;
        return node;
    }

    //parent node
    std::vector<std::vector<uint32_t>> childTriangles(8);
    std::vector<std::vector<uint32_t>> childMeshIndices(8);
    BoundingBox3f childBboxes[8] = {};
    divideBBox(bbox, childBboxes);

    for (int j = 0; j < triangleNum; j++)
    {
        for (int i = 0; i < 8; i++)
        {
            uint32_t triangleIdx = triangleIndices[j];
            uint32_t meshIdx = meshIndices[j];
            BoundingBox3f triangleBBox = m_meshes[meshIdx]->getBoundingBox(triangleIdx);
            if (childBboxes[i].overlaps(triangleBBox))
            {
                childTriangles[i].emplace_back(triangleIdx);
                childMeshIndices[i].emplace_back(meshIdx);
            }

        }
    }

    Node* node = new Node();
    node->bbox = BoundingBox3f(bbox);
    node->child = new Node*[8];
    for (int i = 0; i < 8; i++)
    {
        node->child[i] = buildRecursive(childBboxes[i], childTriangles[i], childMeshIndices[i], recursiveDepth + 1);
        m_recursion_depth = std::max(m_recursion_depth, recursiveDepth + 1);
    }
    return node;
}


void Accel::divideBBox(const BoundingBox3f& parent, BoundingBox3f* childBBox)
{
    Vector3f extents = parent.getExtents();
    float x0 = parent.min.x();
    float x1 = parent.min.x() + extents.x() / 2.0f;
    float x2 = parent.max.x();
    float y0 = parent.min.y();
    float y1 = parent.min.y() + extents.y() / 2.0f;
    float y2 = parent.max.y();
    float z0 = parent.min.z();
    float z1 = parent.min.z() + extents.z() / 2.0f;
    float z2 = parent.max.z();
    Point3f x0y0z0 = Point3f(x0, y0, z0);
    Point3f x1y0z0 = Point3f(x1, y0, z0);
    Point3f x0y1z0 = Point3f(x0, y1, z0);
    Point3f x1y1z0 = Point3f(x1, y1, z0);
    Point3f x0y0z1 = Point3f(x0, y0, z1);
    Point3f x1y0z1 = Point3f(x1, y0, z1);
    Point3f x0y1z1 = Point3f(x0, y1, z1);
    Point3f x1y1z1 = Point3f(x1, y1, z1);
    Point3f x2y1z1 = Point3f(x2, y1, z1);
    Point3f x1y2z1 = Point3f(x1, y2, z1);
    Point3f x2y2z1 = Point3f(x2, y2, z1);
    Point3f x1y1z2 = Point3f(x1, y1, z2);
    Point3f x2y1z2 = Point3f(x2, y1, z2);
    Point3f x1y2z2 = Point3f(x1, y2, z2);
    Point3f x2y2z2 = Point3f(x2, y2, z2);

    childBBox[0] = BoundingBox3f(x0y0z0, x1y1z1);
    childBBox[1] = BoundingBox3f(x1y0z0, x2y1z1);
    childBBox[2] = BoundingBox3f(x0y1z0, x1y2z1);
    childBBox[3] = BoundingBox3f(x1y1z0, x2y2z1);
    childBBox[4] = BoundingBox3f(x0y0z1, x1y1z2);
    childBBox[5] = BoundingBox3f(x1y0z1, x2y1z2);
    childBBox[6] = BoundingBox3f(x0y1z1, x1y2z2);
    childBBox[7] = BoundingBox3f(x1y1z1, x2y2z2);
}


 

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    foundIntersection = traversalIntersect(*m_root, ray, its, shadowRay, f);
    if (shadowRay)
        return foundIntersection;

    /* Brute force search through all triangles */
    //for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
    //    float u, v, t;
    //    if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
    //        /* An intersection was found! Can terminate
    //           immediately if this is a shadow ray query */
    //        if (shadowRay)
    //            return true;
    //        ray.maxt = its.t = t;
    //        its.uv = Point2f(u, v);
    //        its.mesh = m_mesh;
    //        f = idx;
    //        foundIntersection = true;
    //    }
    //}

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}



bool Accel::sortChildToRayDistance(const std::pair<int, float>& a, const std::pair<int, float>& b)
{
    return a.second < b.second;
}

bool Accel::traversalIntersect(const Node& node, Ray3f& ray, Intersection& its, bool shadowRay, uint32_t& hit_idx) const
{
    bool foundIntersection = false;
    if (!node.bbox.rayIntersect(ray))
        return false;
    for (uint32_t idx = 0; idx < node.triangleNum; ++idx)
    {
        uint32_t triangleIdx = node.triangleIndices[idx];
        uint32_t meshIdx = node.meshIndices[idx];
        float u, v, t;
        if (m_meshes[meshIdx]->rayIntersect(triangleIdx, ray, u, v, t) && t < ray.maxt)
        {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_meshes[meshIdx];
            hit_idx = triangleIdx;
            foundIntersection = true;
        }
    }
    if (node.child)
    {
        ////################## Improved ray traversal ################
        ////建立<childIdx, childNodeBBoxToRayDistance>
        //std::vector<std::pair<int, float>> childToRayDistances(8);
        //for (int i = 0; i < 8; i++)
        //{
        //    Node* childNode = node.child[i];
        //    float distance = std::numeric_limits<float>::max();
        //    if (childNode)
        //    {
        //        distance = childNode->bbox.distanceTo(ray.o);
        //    }
        //    childToRayDistances[i] = std::make_pair(i, distance);
        //}
        //sort(childToRayDistances.begin(), childToRayDistances.end(), sortChildToRayDistance);
        ////按到光线起始点距离从近到远遍历
        //for (int i = 0; i < 8; i++)
        //{
        //    int childIndex = childToRayDistances[i].first;
        //    Node* childNode = node.child[childIndex];
        //    if (childNode)
        //    {
        //        foundIntersection = traversalIntersect(*childNode, ray, its, shadowRay, hit_idx) || foundIntersection;
        //    }
        //    if (shadowRay && foundIntersection)
        //        return true;
        //}
        //################## ray traversal ################
        for (int i = 0; i < 8; i++)
        {
            Node* childNode = node.child[i];
            if (childNode)
            {
                foundIntersection = traversalIntersect(*childNode, ray, its, shadowRay, hit_idx) || foundIntersection;
            }
            if (shadowRay && foundIntersection)
                return true;
        }
    }
    return foundIntersection;
}

NORI_NAMESPACE_END

