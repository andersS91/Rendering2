// 02562 Rendering Framework
// Code from GEL (http://www.imm.dtu.dk/GEL/)
// Inspired by Nate Robins Obj loader.
// Modified by Jeppe Revall Frisvad to suit framework 
// and to use CUDA/OptiX matrix-vector types.
// Copyright (c) DTU Informatics 2011

#include <iostream>
#include <cstdio>
#include <string>
#include <optix_world.h>
#include "IndexedFaceSet.h"
#include "HitInfo.h"
#include "Object3D.h"
#include "Triangle.h"
#include "TriMesh.h"

using namespace std;
using namespace optix;

bool TriMesh::intersect(const Ray& r, HitInfo& hit, unsigned int prim_idx) const
{
	const uint3& face = geometry.face(prim_idx);

	// Implement ray-triangle intersection here.
	//
	// Input:  r                    (the ray to be checked for intersection)
	//         prim_idx             (index of the triangle to be tested for intersection)
	//
	// Output: hit.has_hit          (set true if the ray intersects the triangle)
	//         hit.dist             (distance from the ray origin to the intersection point)
	//         hit.geometric_normal (the normalized normal of the triangle)
	//         hit.shading_normal   (the normalized normal of the triangle)
	//         hit.material         (pointer to the material of the triangle)
	//        (hit.texcoord)        (texture coordinates of intersection point)
	//
	// Return: True if the ray intersects the triangle, false otherwise
	//
	// Relevant data fields that are available (see TriMesh.h and above)
	// r                            (the ray)
	// face                         (triangle vertex indices)
	// geometry                     (indexed face set containing vertex positions)
	// normals                      (indexed face set containing vertex normals)
	// texcoords                    (indexed face set containing vertex texture coordinates)
	// mat_idx                      (array containing material index for each triangle)
	// materials                    (array of materials)
	//
	// Hints: (a) Use the function intersect_triangle(...) to get the hit info.
	//        (b) Use the barycentric coordinates of the intersection point
	//        to interpolate the normal (and texture coordinates, if needed)
	//        linearly across the triangle.
	//        (c) Use the function has_normals() to check if the mesh has
	//        vertex normals for computing the shading normal. If not, use
	//        the geometric normal as shading normal.

	const float3 v0 = geometry.vertex(face.x);
	const float3 v1 = geometry.vertex(face.y);
	const float3 v2 = geometry.vertex(face.z);
	const float3 n0 = normals.vertex(face.x);
	const float3 n1 = normals.vertex(face.y);
	const float3 n2 = normals.vertex(face.z);
	float3 n = make_float3(0.0f);
	float tmark = 0.0f;
	float v = 0.0f;
	float w = 0.0f;

	//const uint3& norms = normals.face(prim_idx);
	//const uint3& texCo = texcoords.face(prim_idx);

	if (TriMesh::intersect_triangle(r, v0, v1, v2, n, tmark, v, w)) {
		n = normalize(n0*(1.0f-v-w) + n1*v + n2*w);
		hit.has_hit = true;
		hit.dist = tmark;
		hit.position = r.origin + r.direction * tmark;
		hit.geometric_normal = n;
		hit.shading_normal = n;
		hit.material = &materials[mat_idx[prim_idx]];
		return true;
	}

	return false;
}

bool TriMesh::intersect_triangle(const Ray& ray,
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

void TriMesh::transform(const Matrix4x4& m)
{
	for (unsigned int i = 0; i < geometry.no_vertices(); ++i)
		geometry.vertex_rw(i) = make_float3(m*make_float4(geometry.vertex(i), 1.0f));
	for (unsigned int i = 0; i < normals.no_vertices(); ++i)
		normals.vertex_rw(i) = make_float3(m*make_float4(normals.vertex(i), 0.0f));
}

Aabb TriMesh::get_primitive_bbox(unsigned int prim_idx) const
{
	Aabb bbox;
	uint3 face = geometry.face(prim_idx);
	bbox.include(geometry.vertex(face.x));
	bbox.include(geometry.vertex(face.y));
	bbox.include(geometry.vertex(face.z));
	return bbox;
}

Aabb TriMesh::compute_bbox() const
{
	Aabb bbox;
	for (unsigned int i = 0; i < geometry.no_vertices(); ++i)
		bbox.include(geometry.vertex(i));
	return bbox;
}

unsigned int TriMesh::find_material(const string& name) const
{
	for (unsigned int i = 0; i < materials.size(); ++i)
	{
		if (materials[i].name == name)
			return i;
	}
	return 0;
}

void TriMesh::compute_normals()
{
	// By default the normal faces are the same as the geometry faces
	// and there are just as many normals as vertices, so we simply
	// copy.
	normals = geometry;

	// The normals are initialized to zero.
	for (unsigned int i = 0; i < normals.no_vertices(); ++i)
		normals.vertex_rw(i) = make_float3(0.0f);

	// For each face
	for (unsigned int i = 0; i < geometry.no_faces(); ++i)
	{
		// Compute the normal
		const uint3& f = geometry.face(i);
		const float3& p0 = geometry.vertex(f.x);
		const float3& a = geometry.vertex(f.y) - p0;
		const float3& b = geometry.vertex(f.z) - p0;
		float3 face_normal = cross(a, b);
		float len = dot(face_normal, face_normal);
		if (len > 0.0f)
			face_normal /= sqrt(len);

		// Add the angle weighted normal to each vertex
		const unsigned int* fp = &f.x;
		for (int j = 0; j < 3; ++j)
		{
			const float3& p0 = geometry.vertex(fp[j]);
			float3 a = geometry.vertex(fp[(j + 1) % 3]) - p0;
			float len_a = dot(a, a);
			if (len_a > 0.0f)
				a /= sqrt(len_a);
			float3 b = geometry.vertex(fp[(j + 2) % 3]) - p0;
			float len_b = dot(b, b);
			if (len_b > 0.0f)
				b /= sqrt(len_b);
			float d = optix::fmaxf(-1.0f, optix::fminf(1.0f, dot(a, b)));
			normals.vertex_rw(fp[j]) += face_normal*acos(d);
		}
	}

	// Normalize all normals
	for (unsigned int i = 0; i < normals.no_vertices(); ++i)
	{
		const float3& normal = normals.vertex(i);
		float len_normal = dot(normal, normal);
		if (len_normal > 0.0f)
			normals.vertex_rw(i) /= sqrt(len_normal);
	}
}

void TriMesh::compute_areas()
{
	int no_of_faces = geometry.no_faces();
	surface_area = 0.0f;
	face_areas.resize(no_of_faces);
	face_area_cdf.resize(no_of_faces);
	for (int i = 0; i < no_of_faces; ++i)
	{
		const uint3& f = geometry.face(i);
		const float3& p0 = geometry.vertex(f.x);
		const float3& a = geometry.vertex(f.y) - p0;
		const float3& b = geometry.vertex(f.z) - p0;
		face_areas[i] = 0.5f*length(cross(a, b));
		face_area_cdf[i] = surface_area + face_areas[i];
		surface_area += face_areas[i];
	}
	if (surface_area > 0.0f)
		for (int i = 0; i < no_of_faces; ++i)
			face_area_cdf[i] /= surface_area;
}
