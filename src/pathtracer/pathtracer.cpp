#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
    int num_samples = scene->lights.size() * ns_area_light;
    Vector3D L_out = Vector3D();

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
    for (int i = 0; i < num_samples; i++) {
        Vector3D wi = hemisphereSampler->get_sample();
        Vector3D wi_world = o2w * wi;
        float pdf = 1 / (2 * PI);
        Ray r = Ray(hit_p, wi_world);
        r.min_t = EPS_F;
        Intersection rayIs;
        if (!bvh->intersect(r, &rayIs)) { continue; }
        Vector3D L = rayIs.bsdf->get_emission();
        L_out += L * isect.bsdf->f(w_out, wi) * dot(wi, Vector3D(0, 0, 1)) / pdf;
        // cout << isect.bsdf->get_emission() << endl;
    }
    L_out /= num_samples;
    //cout << L_out << endl;
    return L_out;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);
    Vector3D L_out;
    
    for (SceneLight* light : scene->lights) {
        if (light->is_delta_light()) {
            Vector3D wi;
            double dis;
            double pdf;
            Vector3D radiance = light->sample_L(hit_p, &wi, &dis, &pdf);
            Vector3D wi_obj = w2o * wi;
            if (dot(wi_obj, Vector3D(0,0,1)) < 0) { continue; } // light is behind
            Ray r = Ray(hit_p, wi);
            r.min_t = EPS_F;
            dis -= EPS_F;
            Intersection rayIs;
            if (bvh->intersect(r, &rayIs)) { continue; } // obstacle detected
            L_out += isect.bsdf->f(w_out, wi_obj) * radiance * dot(wi_obj, Vector3D(0,0,1)) / pdf;
        } else {
            Vector3D L_out_area;
            
            for (int i = 0; i < ns_area_light; i++) {
                Vector3D wi;
                double dis;
                double pdf;
                Vector3D radiance = light->sample_L(hit_p, &wi, &dis, &pdf);
                Vector3D wi_obj = w2o * wi;
                if (dot(wi_obj, Vector3D(0,0,1)) < 0) {continue;}
                Ray r = Ray(hit_p, wi);
                r.min_t = EPS_F;
                dis -= EPS_F;
                r.max_t = dis;
                Intersection rayIs;
                if (bvh->intersect(r, &rayIs)) {continue;}
                L_out_area += isect.bsdf->f(w_out, wi_obj) * radiance * dot(wi_obj, Vector3D(0,0,1)) / pdf;
            }
            
            L_out_area /= ns_area_light;
            L_out += L_out_area;
        }
    }
    //cout << L_out << endl;
    return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
    //cout << "outputting:  " << isect.bsdf->get_emission() << endl;
    return isect.bsdf->get_emission();

}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
    if (this->direct_hemisphere_sample) {
        return estimate_direct_lighting_hemisphere(r, isect);
    }
    return estimate_direct_lighting_importance(r, isect);


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D w_out = w2o * (-r.d);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
    Vector3D L_out = one_bounce_radiance(r, isect);
    Vector3D wi_s;
    double pdf;
    Vector3D bsdf_s = isect.bsdf->sample_f(w_out, &wi_s, &pdf);
    
    
    bool russian = coin_flip(0.3);
    // given that russian roulette decides to kill the recursion AND
    // we have already begun recursing (in the case that global illumination is turned on) OR
    // global illumination is just off
        // but when global illumination is off, the depth is already there
    bool maxDepthReached = r.depth <= 1;
    bool globalIlluminationEnabled = max_ray_depth > 1;
    bool recursionStarted = r.depth != max_ray_depth;
    bool shouldStop = (russian && (!globalIlluminationEnabled || recursionStarted)) || maxDepthReached;
    auto russian_roulette = coin_flip(.5)?0.6:0.7;
    bool flag = (coin_flip(1 - russian_roulette) && (this->max_ray_depth <= 1 ||
                   r.depth != this->max_ray_depth)) || (r.depth <= 1);
    if (!shouldStop) {
        Vector3D wi = o2w * wi_s;
        Ray ray = Ray(hit_p,  wi);
        ray.min_t = EPS_F;
        ray.depth = ray.depth - 1;
        Intersection i;
        if (bvh->intersect(ray, &i)) {
            Vector3D radiance = at_least_one_bounce_radiance(ray, i);
            double prob = 1 - 0.3;
            if (r.depth == max_ray_depth) {
                prob = 1.0;
            }
            L_out += bsdf_s * radiance * dot(wi_s, Vector3D(0,0,1)) / pdf / prob;
        }
    }
    return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;
    
  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
    
    
    
    if (!bvh->intersect(r, &isect)) {
        return envLight ? envLight->sample_dir(r) : L_out;
    }


     // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

    // TODO (Part 3): Return the direct illumination.
    //Vector3D zero = zero_bounce_radiance(r, isect);
    Vector3D d = zero_bounce_radiance(r, isect);
    Vector3D id = at_least_one_bounce_radiance(r, isect);
    //Vector3D global = at_least_one_bounce_radiance(r, isect);
    L_out = d + id;
    // TODO (Part 4): Accumulate the "direct" and "indirect"
    // parts of global illumination into L_out rather than just direct
    return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
    // TODO (Part 1.2):
    // Make a loop that generates num_samples camera rays and traces them
    // through the scene. Return the average Vector3D.
    // You should call est_radiance_global_illumination in this function.
    
    // TODO (Part 5):
    // Modify your implementation to include adaptive sampling.
    // Use the command line parameters "samplesPerBatch" and "maxTolerance"
    
        size_t num_samples = ns_aa;          // total samples to evaluate
        Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
        int i = 0;
        Vector3D curTotal = Vector3D();
        float s1 = 0;
        float s2 = 0;
        for (i = 0; i < num_samples; ++i) {
            //cout << i << endl;
            if (i % samplesPerBatch == 0 && i != 0) {
                double n = double(i);
                double mu = s1 / n;
                double alpha = sqrt((s2 - (s1 * s1) / n)/(n - 1.0));
                float I = 1.96 * alpha / sqrt(n);
                if (I <= maxTolerance * mu) {
                    //cout << numConverge << ", " << i << endl;
                    break;
                }
            }

            Vector2D sample = origin + gridSampler->get_sample();
            float x_norm = sample.x / sampleBuffer.w;
            float y_norm = sample.y / sampleBuffer.h;
            Ray sampleRay = camera->generate_ray(x_norm, y_norm);
            sampleRay.depth = max_ray_depth;
            Vector3D radiance = est_radiance_global_illumination(sampleRay);
    
            float x_k = radiance.illum();
            s1 += x_k;
            s2 += x_k * x_k;
            curTotal += radiance;
            }
        sampleCountBuffer[x + y * sampleBuffer.w] = i;
        sampleBuffer.update_pixel(curTotal / double(i), x, y);
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
