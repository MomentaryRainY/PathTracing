//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <atomic>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 4;
    std::cout << "SPP: " << spp << "\n";

     // 确定使用的线程数，通常与硬件并发数相同
    unsigned int numThreads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads(numThreads);

    // Lambda 表达式，用于并行处理每个像素
    auto renderPixel = [&](int start, int end) {
        for (int m = start; m < end; ++m) {
            int j = m / scene.width;
            int i = m % scene.width;
            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            Vector3f dir = normalize(Vector3f(-x, y, 1));
            Ray ray = Ray(eye_pos, dir);
            Vector3f color(0, 0, 0); // 初始化颜色累加器
            for (int k = 0; k < spp; k++){
                color += scene.castRay(ray, 0);
            }
            framebuffer[m] = color / spp;
            // 更新进度
            pixelsRendered++;
            if (pixelsRendered.load() % 1000 == 0) {
                UpdateProgress(pixelsRendered / (float)(scene.width * scene.height));
            }
        }
    };

    // 分配任务给每个线程
    int pixelsPerThread = (scene.width * scene.height) / numThreads;
    for (unsigned int i = 0; i < numThreads; ++i) {
        int start = i * pixelsPerThread;
        int end = (i + 1 == numThreads) ? scene.width * scene.height : (i + 1) * pixelsPerThread;
        threads[i] = std::thread(renderPixel, start, end);
    }

    for (auto& thread : threads) {
        thread.join(); // 等待所有线程完成
    }

    UpdateProgress(1.f); // 完成渲染，更新进度

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

void Renderer::UpdateProgress(float progress) {
    static std::mutex progressMutex;
    std::lock_guard<std::mutex> lock(progressMutex);
    std::cout << "\rRendering Progress: " << progress * 100 << "%        " << std::flush;
}
