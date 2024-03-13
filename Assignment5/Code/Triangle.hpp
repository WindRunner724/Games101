#pragma once

#include "Object.hpp"
#include <eigen3/Eigen/Eigen>

#include <cstring>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    
    //Vector3f normal = crossProduct(v1 - v0, v2 - v0);
    //Vector3f x = dotProduct(dir, normal);
    //Vector3f b = dotProduct(v0 - orig, normal);
    //
    //float coff = 0.0;
    //if(x.x!=0) coff = b.x / x.x;
    //else if(x.y!=0) coff = b.y / x.y;
    //else coff = b.z / x.z;
    //Vector3f inter = orig + dir * coff;
    //// Check inter pointer whether is in triangle
    //Vector3f cross1 = crossProduct(v0 - inter, v1 - inter);
    //Vector3f cross2 = crossProduct(v1 - inter, v2 - inter);
    //Vector3f cross3 = crossProduct(v2 - inter, v0 - inter);

    //if (coff >= 0 && dotProduct(cross1, cross2) > 0 && dotProduct(cross2, cross3) > 0)
    //{
    //    Vector3f e0 = v1 - v0;
    //    Vector3f e1 = v2 - v0;
    //    Vector3f e2 = inter - v0;

    //    double d00 = dotProduct(e0, e0);
    //    double d01 = dotProduct(e0, e1);
    //    double d11 = dotProduct(e1, e1);
    //    double d20 = dotProduct(e2, e0);
    //    double d21 = dotProduct(e2, e1);
    //    double denom = d00 * d11 - d01 * d01;

    //    double b0 = (d11 * d20 - d01 * d21) / denom;
    //    double b1 = (d00 * d21 - d01 * d20) / denom;
    //    double b2 = 1.0 - b0 - b1;
    //    tnear = coff;
    //    u = b1;
    //    v = b2;
    //    return true;
    //}
    //return false;

    auto e1 = v1 - v0, e2 = v2 - v0, s = orig - v0;
    auto s1 = crossProduct(dir, e2), s2 = crossProduct(s, e1);

    float t = dotProduct(s2, e2) / dotProduct(s1, e1);
    float b1 = dotProduct(s1, s) / dotProduct(s1, e1);
    float b2 = dotProduct(s2, dir) / dotProduct(s1, e1);

    if (t > 0.0 && b1 > 0.0 && b2 > 0.0 && (1 - b1 - b2) > 0.0)
    {
        tnear = t;
        u = b1;
        v = b2;

        return true;
    }
    return false;

}

class MeshTriangle : public Object
{
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)// verts: 顶点列表，vertsIndex: 归属三角形的顶点序号，numTris: 三角形数量， st
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;// 为顶点列表划出范围
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index,
                   Vector2f& uv) const override
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
                              Vector2f& st) const override
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
