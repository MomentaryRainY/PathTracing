//
// Created by goksu on 2/25/20.
//
#include "Scene.hpp"

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
    void UpdateProgress(float progress);
private:
    std::atomic<int> pixelsRendered{0}; // 使用原子操作来统计已渲染的像素数，保证线程安全
};
