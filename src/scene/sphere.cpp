#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
    double a = dot(r.d, r.d);
    double b = 2.0 * dot((r.o - o), r.d);
    double c = dot(r.o - o, r.o - o) - r2;
    double root = pow(b, 2.0) - 4.0 * a * c;
    if (root < 0) {
        return false;
    }
    double r1 = (-b + sqrt(root)) / (2.0 * a);
    double r2 = (-b - sqrt(root)) / (2.0 * a);
    if (r1 < r.min_t || r1 > r.max_t) {return false;}
    if (r2 < r.min_t || r2 > r.max_t) {return false;}
    t1 = min(r1, r2);
    t2 = max(r1, r2);
    r.max_t = t1;
    return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here
    double t1 = 0.0;
    double t2 = 0.0;
    return test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    double t1 = 0.0;
    double t2 = 0.0;
    if (test(r, t1, t2)) {
        i->t = t1;
        Vector3D normal = r.at_time(t1) - o;
        normal.normalize();
        i->n = normal;
        i->primitive = this;
        i->bsdf = get_bsdf();
        return true;
    }
    return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
