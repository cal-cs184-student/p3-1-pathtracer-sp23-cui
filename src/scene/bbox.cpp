#include "bbox.h"

#include "GL/glew.h"
#include <vector>
#include <algorithm>
#include <iostream>

namespace CGL {

void swap(double& d1, double& d2) {
    double temp = d1;
    d1 = d2;
    d2 = temp;
}

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
    
    // TODO (Part 2.2):
    Vector3D tmax = Vector3D();
    Vector3D tmin = Vector3D();
    double abs_min;
    double abs_max;
    tmax.x = (max.x - r.o.x) / r.d.x;
    tmin.x = (min.x - r.o.x) / r.d.x;
    if (r.d.x < 0) swap(tmax.x, tmin.x);
    abs_min = tmin.x;
    abs_max = tmax.x;
    
    tmax.y = (max.y - r.o.y) / r.d.y;
    tmin.y = (min.y - r.o.y) / r.d.y;
    if (r.d.y < 0) swap(tmax.y, tmin.y);
    
    if (tmin.x > tmax.y || tmin.y > tmax.x) return false;
    if (tmin.y > tmin.x) abs_min = tmin.y;
    if (tmax.y < tmax.x) abs_max = tmax.y;
    
    tmax.z = (max.z - r.o.z) / r.d.z;
    tmin.z = (min.z - r.o.z) / r.d.z;
    if (r.d.z < 0) swap(tmax.z, tmin.z);
    
    if (abs_min > tmax.z || tmin.z > abs_max) return false;
    if (tmin.z > abs_min) abs_min = tmin.z;
    if (tmax.z < abs_max) abs_max = tmax.z;
    if (abs_min < t1 && abs_max > t0) {
        return true;
    }
}

void BBox::draw(Color c, float alpha) const {
    
    glColor4f(c.r, c.g, c.b, alpha);
    
    // top
    glBegin(GL_LINE_STRIP);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(max.x, max.y, max.z);
    glEnd();
    
    // bottom
    glBegin(GL_LINE_STRIP);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, min.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glEnd();
    
    // side
    glBegin(GL_LINES);
    glVertex3d(max.x, max.y, max.z);
    glVertex3d(max.x, min.y, max.z);
    glVertex3d(max.x, max.y, min.z);
    glVertex3d(max.x, min.y, min.z);
    glVertex3d(min.x, max.y, min.z);
    glVertex3d(min.x, min.y, min.z);
    glVertex3d(min.x, max.y, max.z);
    glVertex3d(min.x, min.y, max.z);
    glEnd();
    
}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
    return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
