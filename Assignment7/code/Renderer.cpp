//
// Created by goksu on 2/25/20.
//

#pragma warning(disable: 4244)
#pragma warning(disable: 4305)
#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <omp.h>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }
const float EPSILON = 0.01;

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
    //int spp = 16;
    //std::cout << "SPP: " << spp << "\n";
    //for (uint32_t j = 0; j < scene.height; ++j) {
    //    for (uint32_t i = 0; i < scene.width; ++i) {
    //        // generate primary ray direction
    //        float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                  imageAspectRatio * scale;
    //        float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //        Vector3f dir = normalize(Vector3f(-x, y, 1));
    //        for (int k = 0; k < spp; k++){
    //            framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //        }
    //        m++;
    //    }
    //    UpdateProgress(j / (float)scene.height);
    //}
    //UpdateProgress(1.f);

    //OMP Multithreads
    int32_t progress = 0;
    int32_t currProgress = 0;
    const int spp = 16000;
    const int msaaNum = 64;
    const int32_t totalSize = scene.width * scene.height;
    std::cout << "SPP: " << spp << "  MSAA: "<< msaaNum << "\n";

    omp_lock_t lock1;
    omp_init_lock(&lock1);
    UpdateProgress(0.0f);
    #pragma omp parallel for
    for (int32_t index = 0; index < totalSize; index++)
    {
        int i, j = 0;
        currProgress++;
        getCoord(index, scene.width, i, j);
        // generate primary ray direction
        std::vector<float> x(msaaNum);
        std::vector<float> y(msaaNum);
        Vector3f dir[msaaNum];
        if (msaaNum % 4 == 0)
            MSAAx(i + 0.5, j + 0.5, 1.0f, x, y, 0, msaaNum);
        else
            throw std::runtime_error("Wrong MSAA setting!");
        for (int i = 0; i < msaaNum; i++) {
            float x_local = (2 * x[i] / (float)scene.width - 1) * imageAspectRatio * scale;
            float y_local = (1 - 2 * y[i] / (float)scene.height) * scale;
            dir[i] = normalize(Vector3f(-x_local, y_local, 1));
        }

        int cnt = 0;
        for (int k = 0; k < spp; k++)
        {
            if (cnt == msaaNum) cnt = 0;
            framebuffer[index] += scene.castRay(Ray(eye_pos, dir[cnt++]), 0);
        }
        framebuffer[index] = framebuffer[index] / spp;

        float buffProgress = static_cast<float>(currProgress) * 10 / totalSize;
        if (buffProgress >= progress+1) {
            omp_set_lock(&lock1);
            progress = buffProgress;
            UpdateProgress((float)progress/10);
            omp_unset_lock(&lock1);
        }
    }
    UpdateProgress(1.0f);
    omp_destroy_lock(&lock1);
    // save framebuffer to file
    FILE* fp;
    if (fopen_s(&fp, "binary.ppm", "wb") == 0) {
        (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
        for (auto i = 0; i < scene.height * scene.width; ++i) {
            static unsigned char color[3];
            color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
            color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
            color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
            fwrite(color, 1, 3, fp);
        }
        fclose(fp);
    }
    else {
        std::cout << "File open fail";
        return;
    }
}
