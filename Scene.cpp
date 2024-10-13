//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <typeinfo>


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
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

// // Implementation of Path Tracing
// Vector3f Scene::castRay(const Ray &ray, int depth) const
// {  
//     // TO DO Implement Path Tracing Algorithm here

//     Vector3f hitColor = this->backgroundColor;
    
//     Intersection interP = Scene::intersect(ray);
//     if (!interP.happened) return hitColor;

//     Intersection interL;
//     float pdf_light;
//     sampleLight(interL, pdf_light);

//     Vector3f L_indir(0), L_dir(0);

//     //Get x, ws, NN, emit from inter
//     Vector3f wo = ray.direction;
//     Vector3f p = interP.coords;
//     Vector3f x = interL.coords;
//     Vector3f ws = normalize(x - p);
//     Vector3f NN = interP.normal;
//     Vector3f emit = interL.emit;
//     Vector3f N = interL.normal;

//     float distance = (x-p).norm();
    
//     //Shoot a ray from p to x
//     Ray rayx(p, ws);
//     //If the ray is not blocked in the middle
//     Intersection interX = Scene::intersect(rayx);
//     if (abs(distance - interX.distance < 0.01)) {
//         L_dir = emit * interP.m->eval(wo, ws, NN) * dotProduct(ws, NN) * dotProduct(-ws, N) / distance / distance / pdf_light;
//     }
        
//     //Test Russian Roulette with probability RussianRoulette
//     if (get_random_float() < RussianRoulette) {
        
//         Vector3f wi = normalize(interP.m->sample(wo, NN));
//         //Trace a ray r(p, wi)
//         Ray rayr(p, wi);
//         //If ray r hit a non -emitting object at q
//         Intersection interR = Scene::intersect(rayr);
//         if (interR.happened && !interR.m->hasEmission())
//             L_indir = castRay(rayr, depth + 1) * interP.m->eval(wo, wi, NN) * dotProduct(wi, NN) / interP.m->pdf(wo, wi, NN) / RussianRoulette;
        
//         hitColor = interP.m->getEmission() + L_dir + L_indir;
//     }
    
//     return hitColor;
// }

Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f hitColor = this->backgroundColor;
    Intersection shade_point_inter = Scene::intersect(ray);
    if (shade_point_inter.happened)
    {

        Vector3f p = shade_point_inter.coords;
        Vector3f wo = ray.direction;
        Vector3f N = shade_point_inter.normal;
        Vector3f L_dir(0), L_indir(0);

       //sampleLight(inter,pdf_light)
        Intersection light_point_inter;
        float pdf_light;
        sampleLight(light_point_inter, pdf_light);
        //Get x,ws,NN,emit from inter
        Vector3f x = light_point_inter.coords;
        Vector3f ws = normalize(x-p);
        Vector3f NN = light_point_inter.normal;
        Vector3f emit = light_point_inter.emit;
        float distance_pTox = (x - p).norm();
        //Shoot a ray from p to x
        Vector3f p_deviation = (dotProduct(ray.direction, N) < 0) ?
                p + N * EPSILON :
                p - N * EPSILON ;

        Ray ray_pTox(p_deviation, ws);
        //If the ray is not blocked in the middleff
        Intersection blocked_point_inter = Scene::intersect(ray_pTox);
        if (abs(distance_pTox - blocked_point_inter.distance < 0.01 ))
        {
            L_dir = emit * shade_point_inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (distance_pTox * distance_pTox * pdf_light);
        }
        //Test Russian Roulette with probability RussianRouolette
        float ksi = get_random_float();
        if (ksi < RussianRoulette)
        {
            //wi=sample(wo,N)
            Vector3f wi = normalize(shade_point_inter.m->sample(wo, N));
            //Trace a ray r(p,wi)
            Ray ray_pTowi(p_deviation, wi);
            //If ray r hit a non-emitting object at q
            Intersection bounce_point_inter = Scene::intersect(ray_pTowi);
            if (bounce_point_inter.happened && !bounce_point_inter.m->hasEmission())
            {
                float pdf = shade_point_inter.m->pdf(wo, wi, N);
                if(pdf> EPSILON)
                    L_indir = castRay(ray_pTowi, depth + 1) * shade_point_inter.m->eval(wo, wi, N) * dotProduct(wi, N) / (pdf *RussianRoulette);
            }
        }
        hitColor = shade_point_inter.m->getEmission() + L_dir + L_indir;
    }
    return hitColor;
}