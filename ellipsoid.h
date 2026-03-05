#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include "hittable.h"
#include "onb.h"

class ellipsoid : public hittable {
public:
    ellipsoid(point3 cen,
        double rx, double ry, double rz,
        shared_ptr<material> m)
        : center(cen),
        radius_x(rx), radius_y(ry), radius_z(rz),
        mat(m)
    {
        bbox = aabb(
            point3(center.x() - rx, center.y() - ry, center.z() - rz),
            point3(center.x() + rx, center.y() + ry, center.z() + rz)
        );
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {

        vec3 oc = r.origin() - center;

        // Transform ray into unit sphere space
        vec3 d(
            r.direction().x() / radius_x,
            r.direction().y() / radius_y,
            r.direction().z() / radius_z
        );

        vec3 o(
            oc.x() / radius_x,
            oc.y() / radius_y,
            oc.z() / radius_z
        );

        double a = d.length_squared();
        double half_b = dot(o, d);
        double c = o.length_squared() - 1.0;

        double discriminant = half_b * half_b - a * c;
        if (discriminant < 0)
            return false;

        double sqrtd = sqrt(discriminant);

        double root = (-half_b - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (-half_b + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);

        vec3 unit_p(
            (rec.p.x() - center.x()) / radius_x,
            (rec.p.y() - center.y()) / radius_y,
            (rec.p.z() - center.z()) / radius_z
        );

        unit_p = unit_vector(unit_p);

        get_sphere_uv(unit_p, rec.u, rec.v);

        // Proper ellipsoid normal (gradient)
        vec3 outward_normal(
            (rec.p.x() - center.x()) / (radius_x * radius_x),
            (rec.p.y() - center.y()) / (radius_y * radius_y),
            (rec.p.z() - center.z()) / (radius_z * radius_z)
        );

        outward_normal = unit_vector(outward_normal);
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;

        return true;
    }

    aabb bounding_box() const override {
        return bbox;
    }

private:
    point3 center;
    double radius_x, radius_y, radius_z;
    shared_ptr<material> mat;
    aabb bbox;

    static void get_sphere_uv(const point3& p, double& u, double& v) {
        // p: a given point on the sphere of radius one, centered at the origin.
        // u: returned value [0,1] of angle around the Y axis from X=-1.
        // v: returned value [0,1] of angle from Y=-1 to Y=+1.
        //     <1 0 0> yields <0.50 0.50>       <-1  0  0> yields <0.00 0.50>
        //     <0 1 0> yields <0.50 1.00>       < 0 -1  0> yields <0.50 0.00>
        //     <0 0 1> yields <0.25 0.50>       < 0  0 -1> yields <0.75 0.50>

        auto theta = std::acos(-p.y());
        auto phi = std::atan2(-p.z(), p.x()) + pi;

        u = phi / (2 * pi);
        v = theta / pi;
    }
};

#endif