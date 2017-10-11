// 02562 Rendering Framework
// Written by Jeppe Revall Frisvad, 2011
// Copyright (c) DTU Informatics 2011

#include <optix_world.h>
#include "HitInfo.h"
#include "Triangle.h"

using namespace optix;

bool Triangle::intersect_triangle(const Ray& ray,
	const float3& v0,
	const float3& v1,
	const float3& v2,
	float3& n,
	float& t,
	float& v,
	float& w) const
{
	// Implement ray-triangle intersection here (see Listing 1 in the lecture note).
	// Note that OptiX also has an implementation, so you can get away
	// with not implementing this function. However, I recommend that
	// you implement it for completeness.
	static const float EPSILON = 1e-6f;
	const float3 e1 = v1 - v0;
	const float3 e2 = v2 - v0;
	n = normalize(cross(e1, e2));

	const float3 q = cross(ray.direction, e2);

	const float a = dot(e1, q);
	if (a > -EPSILON && a < EPSILON) {
		return false;
	}

	const float3 s = ray.origin - v0;
	const float3 r = cross(s, e1);

	v = dot(s, q) / a;
	w = dot(ray.direction, r) / a;

	const float u = 1.0f - (v + w);
	if (u < 0.0f || u > 1.0f) {
		return false;
	}
	if (v < 0.0f || u + v > 1.0f) {
		return false;
	}

	t = dot(e2, r) * (1.0f / a);
	if (t < ray.tmin || t > ray.tmax) {
		return false;
	}
	return true;

}


bool Triangle::intersect(const Ray& r, HitInfo& hit, unsigned int prim_idx) const
{
	// Implement ray-triangle intersection here.
	//
	// Input:  r                    (the ray to be checked for intersection)
	//         prim_idx             (index of the primitive element in a collection, not used here)
	//
	// Output: hit.has_hit          (set true if the ray intersects the triangle)
	//         hit.dist             (distance from the ray origin to the intersection point)
	//         hit.position         (coordinates of the intersection point)
	//         hit.geometric_normal (the normalized normal of the triangle)
	//         hit.shading_normal   (the normalized normal of the triangle)
	//         hit.material         (pointer to the material of the triangle)
	//        (hit.texcoord)        (texture coordinates of intersection point, not needed for Week 1)
	//
	// Return: True if the ray intersects the triangle, false otherwise
	//
	// Relevant data fields that are available (see Triangle.h)
	// r                            (the ray)
	// v0, v1, v2                   (triangle vertices)
	// (t0, t1, t2)                 (texture coordinates for each vertex, not needed for Week 1)
	// material                     (material of the triangle)
	//
	// Hint: Use the function intersect_triangle(...) to get the hit info.
	//       Note that you need to do scope resolution (optix:: or just :: in front
	//       of the function name) to choose between the OptiX implementation and
	//       the function just above this one.

	float3 n = make_float3(0.0f);
	float tmark = 0.0f;
	float v = 0.0f;
	float w = 0.0f;

	if (Triangle::intersect_triangle(r, v0, v1, v2, n, tmark, v, w)) {
		tmark = fmaxf(tmark, 0.0f);
		hit.has_hit = true;
		hit.dist = tmark;
		hit.position = r.origin + r.direction * tmark;
		hit.geometric_normal = n;
		hit.shading_normal = n;
		hit.material = &material;
		return true;
	}

	return false;
}

void Triangle::transform(const Matrix4x4& m)
{
	v0 = make_float3(m*make_float4(v0, 1.0f));
	v1 = make_float3(m*make_float4(v1, 1.0f));
	v2 = make_float3(m*make_float4(v2, 1.0f));
}

Aabb Triangle::compute_bbox() const
{
	Aabb bbox;
	bbox.include(v0);
	bbox.include(v1);
	bbox.include(v2);
	return bbox;
}
