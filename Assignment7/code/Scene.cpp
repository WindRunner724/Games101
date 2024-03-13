//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma warning(disable: 4244)
#pragma warning(disable: 4305)
#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection isec = intersect(ray);
    if (!isec.happened || depth > this->maxDepth) return Vector3f();
    if (isec.m->hasEmission()) return isec.m->getEmission();

    float pdf;
    Intersection isec_reflect;
    Vector3f L_dir, L_indir;
    Vector3f vec_p2x, vec_x2p;
    // Direct illumination
    if (true)
    {
        sampleLight(isec_reflect, pdf);
        vec_p2x = normalize(isec_reflect.coords - isec.coords);
        vec_x2p = -vec_p2x;
        Vector3f diff = (dotProduct(ray.direction * isec.normal) > 0) ? -isec.normal : isec.normal;
        Intersection isec_reflect_check = intersect(Ray(isec.coords, vec_p2x));
        //if (isec_reflect_check.coords.norm()<EPSILON || 
        //    (isec_reflect_check.coords-isec.coords).norm()-(isec_reflect.coords-isec.coords).norm() > -EPSILON)
        if(isec_reflect_check.coords.norm() < EPSILON || isec_reflect_check.distance - (isec_reflect.coords-isec.coords).norm() > -EPSILON)
        {
            Vector3f L_i = isec_reflect.emit;
            Vector3f f_r = isec.m->eval(ray.direction, vec_p2x, isec.normal);
            float cos_theta = dotProduct(isec.normal, vec_p2x);
            float cos_theta_x = dotProduct(isec_reflect.normal, vec_x2p);

            L_dir = L_i * f_r * cos_theta * cos_theta_x / dotProduct(isec.coords - isec_reflect.coords) / pdf;
        }
    }
    // Indirect illumination
    if (get_random_float() < RussianRoulette)
    {
        vec_p2x = normalize(isec.m->sample(ray.direction, isec.normal));
        vec_x2p = -vec_p2x;
        isec_reflect = intersect(Ray(isec.coords, vec_p2x));
        if (isec_reflect.happened && !isec_reflect.m->hasEmission())
        {
            Vector3f L_i = castRay(Ray(isec.coords, vec_p2x), depth+1);
            Vector3f f_r = isec.m->eval(ray.direction, vec_p2x, isec.normal);
            float cos_theta = dotProduct(isec.normal, vec_p2x);
            //float cos_theta_x = dotProduct(isec_reflect.normal, vec_x2p);
            pdf = isec.m->pdf(ray.direction, vec_p2x, isec.normal);

            if(pdf > EPSILON) 
                L_indir = L_i * f_r * cos_theta / pdf / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}