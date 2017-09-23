// 02562 Rendering Framework
// Written by Jeppe Revall Frisvad, 2011
// Copyright (c) DTU Informatics 2011

#include <optix_world.h>
#include "HitInfo.h"
#include "ObjMaterial.h"
#include "fresnel.h"
#include "RayTracer.h"

using namespace optix;

bool RayTracer::trace_reflected(const Ray& in, const HitInfo& in_hit, Ray& out, HitInfo& out_hit) const
{
	// Initialize the reflected ray and trace it.
	//
	// Input:  in         (the ray to be reflected)
	//         in_hit     (info about the ray-surface intersection)
	//
	// Output: out        (the reflected ray)
	//         out_hit    (info about the reflected ray)
	//
	// Return: true if the reflected ray hit anything
	//
	// Hints: (a) There is a reflect function available in the OptiX math library.
	//        (b) Set out_hit.ray_ior and out_hit.trace_depth.

	float3 reflectedDir = optix::reflect(in.direction, in_hit.shading_normal);

	out = Ray(in_hit.position, reflectedDir, 0, 0.01f);

	if (this->trace_to_closest(out, out_hit))
	{
		out_hit.trace_depth = in_hit.trace_depth + 1;

		out_hit.ray_ior = in_hit.ray_ior;

		return true;
	}

	return false;
}

bool RayTracer::trace_refracted(const Ray& in, const HitInfo& in_hit, Ray& out, HitInfo& out_hit) const
{
	// Initialize the refracted ray and trace it.
	//
	// Input:  in         (the ray to be refracted)
	//         in_hit     (info about the ray-surface intersection)
	//
	// Output: out        (the refracted ray)
	//         out_hit    (info about the refracted ray)
	//
	// Return: true if the refracted ray hit anything
	//
	// Hints: (a) There is a refract function available in the OptiX math library.
	//        (b) Set out_hit.ray_ior and out_hit.trace_depth.
	//        (c) Remember that the function must handle total internal reflection.
	float theta;
	float3 outDir, outNorm, refractedDir;

	out_hit.ray_ior = get_ior_out(in, in_hit, outDir, outNorm, theta);

	if (optix::refract(refractedDir, -outDir, outNorm, out_hit.ray_ior / in_hit.ray_ior))
	{
		out.origin = in_hit.position;
		out.direction = refractedDir;
		out.tmin = 1e-4f;
		out.tmax = 999999;

		if (this->trace_to_closest(out, out_hit))
		{
			out_hit.trace_depth = in_hit.trace_depth + 1;

			return true;
		}
	}

	return false;
}

bool RayTracer::trace_refracted(const Ray& in, const HitInfo& in_hit, Ray& out, HitInfo& out_hit, float& R) const
{
	// Initialize the refracted ray and trace it.
	// Compute the Fresnel reflectance (see fresnel.h) and return it in R.
	//
	// Input:  in         (the ray to be refracted)
	//         in_hit     (info about the ray-surface intersection)
	//
	// Output: out        (the refracted ray)
	//         out_hit    (info about the refracted ray)
	//
	// Return: true if the refracted ray hit anything
	//
	// Hints: (a) There is a refract function available in the OptiX math library.
	//        (b) Set out_hit.ray_ior and out_hit.trace_depth.
	//        (c) Remember that the function must handle total internal reflection.
	R = 0.1;
	return trace_refracted(in, in_hit, out, out_hit);
}

float RayTracer::get_ior_out(const Ray& in, const HitInfo& in_hit, float3& dir, float3& normal, float& cos_theta_in) const
{
	normal = in_hit.shading_normal;
	dir = -in.direction;
	cos_theta_in = dot(normal, dir);
	if (cos_theta_in < 0.0)
	{
		normal = -normal;
		cos_theta_in = -cos_theta_in;
		return 1.0f;
	}
	const ObjMaterial* m = in_hit.material;
	return m ? m->ior : 1.0f;
}
