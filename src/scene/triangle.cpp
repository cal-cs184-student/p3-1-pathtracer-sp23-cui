#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }


Vector3D moller_trumbore(Vector3D p0, Vector3D p1, Vector3D p2, const Ray &r) {
    Vector3D e1 = p1 - p0;
    Vector3D e2 = p2 - p0;
    Vector3D s = r.o - p0;
    Vector3D s1 = cross(r.d, e2);
    Vector3D s2 = cross(s, e1);
    double coefficient = 1.0 / dot(s1, e1);
    Vector3D res = coefficient * Vector3D(dot(s2, e2), dot(s1, s), dot(s2, r.d));
    return res;
}

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
    Vector3D normal = cross(p2-p1, p3-p1);
    Vector3D isect = moller_trumbore(p1, p2, p3, r);
    if (isect.x < r.min_t || isect.x > r.max_t) return false;
    if (isect.y < 0 || isect.y > 1) return false;
    if (isect.z < 0 || isect.y + isect.z > 1) return false;
    return true;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
    if (has_intersection(r)) {
        Vector3D p = moller_trumbore(p1, p2, p3, r);
        isect->t = p.x;
        Vector3D b_coord = Vector3D(1.0 - p.y - p.z, p.y, p.z);
        isect->n = b_coord.x * n1 + b_coord.y * n2 + b_coord.z * n3;
        isect->primitive = this;
        isect->bsdf = get_bsdf();
        r.max_t = p.x;
        return true;
    }
    return false;
     
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
