//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened)
        return {};
    Vector3f L_dir = 0.0, L_indir = 0.0;
    Material *m = intersection.m;

    // 直接光照
    float pdf_light;
    Intersection light_inter;
    sampleLight(light_inter, pdf_light);
    Vector3f light_col = light_inter.coords - intersection.coords;
    float light_dis = light_col.norm();
    Vector3f light_dir = normalize(light_col);
    Ray light_ray(intersection.coords, light_dir);
    Intersection ray_inter = Scene::intersect(light_ray);
    if (ray_inter.distance - light_dis > -EPSILON)
    {
        Vector3f f_r = m->eval(ray.direction, light_dir, intersection.normal);
        L_dir = light_inter.emit * f_r * dotProduct(light_dir, intersection.normal)
                * dotProduct(-light_dir, light_inter.normal) / powf(light_dis, 2) / pdf_light;
    }

    // 间接光照
    // 俄罗斯转盘作为终止条件，为了期望和不做转盘结果一致，还需要除以通过的概率
    if (get_random_float() < RussianRoulette)
    {
        Vector3f wi = m->sample(ray.direction, intersection.normal);
        Ray p_wi(intersection.coords, wi);
        Intersection obj_inter = Scene::intersect(p_wi);
        if (obj_inter.happened && !obj_inter.m->hasEmission())
        {
            Vector3f f_r = m->eval(ray.direction, wi, intersection.normal);
            L_indir = castRay(p_wi, depth + 1) * f_r * dotProduct(wi, intersection.normal)
                    / m->pdf(ray.direction, wi, intersection.normal) / RussianRoulette;
        }
    }

    return L_dir + L_indir + m->getEmission();
}
