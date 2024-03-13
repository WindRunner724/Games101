//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"
#include <cassert>

#pragma once
struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};

inline void sample(const Scene& scene, std::vector<Vector3f>& framebuffer, float scale)
{

}

inline void getCoord(int index, int width, int& i, int& j)
{
    i = index % width;
    j = index / width;
}

//inline void MSAA4(float x, float y, float scale, std::vector<float>& msaa_x, std::vector<float>& msaa_y, int startId)
//{
//    assert(msaa_x.size >= startId + 4 && msaa_y.size >= startId + 4);
//    msaa_x[startId] = x - scale / 4;
//    msaa_x[startId+1] = x + scale / 4;
//    msaa_x[startId+2] = x - scale / 4;
//    msaa_x[startId+3] = x + scale / 4;
//
//    msaa_y[startId] = y + scale / 4;
//    msaa_y[startId+1] = y + scale / 4;
//    msaa_y[startId+2] = y - scale / 4;
//    msaa_y[startId+3] = y - scale / 4;
//}
//
//inline void MSAA16(float x, float y, float scale, std::vector<float>& msaa_x, std::vector<float>& msaa_y, int startId)
//{
//    assert(msaa_x.size >= startId + 16 && msaa_y.size >= startId + 16);
//    MSAA4(x - scale / 4, y + scale / 4, scale / 2, msaa_x, msaa_y, startId);
//    MSAA4(x + scale / 4, y + scale / 4, scale / 2, msaa_x, msaa_y, startId + 4);
//    MSAA4(x - scale / 4, y - scale / 4, scale / 2, msaa_x, msaa_y, startId + 8);
//    MSAA4(x + scale / 4, y - scale / 4, scale / 2, msaa_x, msaa_y, startId + 12);
//}

inline void MSAAx(float x, float y, float scale, std::vector<float>& msaa_x, std::vector<float>& msaa_y, int startId, int times)
{
    assert(msaa_x.size >= startId + times && msaa_y.size >= startId + times);
    assert(times % 4 == 0);
    if (times == 0)
    {
        return;
    }
    else if (times == 4)
    {
        msaa_x[startId] = x - scale / 4;
        msaa_x[startId + 1] = x + scale / 4;
        msaa_x[startId + 2] = x - scale / 4;
        msaa_x[startId + 3] = x + scale / 4;

        msaa_y[startId] = y + scale / 4;
        msaa_y[startId + 1] = y + scale / 4;
        msaa_y[startId + 2] = y - scale / 4;
        msaa_y[startId + 3] = y - scale / 4;
    }
    else
    {
        MSAAx(x - scale / 4, y + scale / 4, scale / 2, msaa_x, msaa_y, startId, times/4);
        MSAAx(x + scale / 4, y + scale / 4, scale / 2, msaa_x, msaa_y, startId + times/4, times/4);
        MSAAx(x - scale / 4, y - scale / 4, scale / 2, msaa_x, msaa_y, startId + times/4*2, times/4);
        MSAAx(x + scale / 4, y - scale / 4, scale / 2, msaa_x, msaa_y, startId + times/4*3, times/4);
    }
}
