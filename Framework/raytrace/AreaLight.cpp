// 02562 Rendering Framework
// Written by Jeppe Revall Frisvad, 2011
// Copyright (c) DTU Informatics 2011

#include <optix_world.h>
#include "IndexedFaceSet.h"
#include "ObjMaterial.h"
#include "mt_random.h"
#include "cdf_bsearch.h"
#include "HitInfo.h"
#include "AreaLight.h"

using namespace optix;

bool AreaLight::sample(const float3& pos, float3& dir, float3& L) const

{
  const IndexedFaceSet& normals = mesh->normals;
  L = make_float3(0.0f);

  // Compute output and return value given the following information.
  //
  // Input:  pos  (the position of the geometry in the scene)
  //
  // Output: dir  (the direction toward the light)
  //         L    (the radiance received from the direction dir)
  //
  // Return: true if not in shadow
  //
  // Relevant data fields that are available (see Light.h and above):
  // shadows             (on/off flag for shadows)
  // tracer              (pointer to ray tracer)
  // normals             (indexed face set of vertex normals)
  // mesh->face_areas    (array of face areas in the light source)
  //
  // Hints: (a) Use mesh->compute_bbox().center() to get the center of 
  //        the light source bounding box.
  //        (b) Use the function get_emission(...) to get the radiance
  //        emitted by a triangle in the mesh.
  static const float EPSILON = 1.0e-4f;
  const float no_of_faces = static_cast<float>(mesh->geometry.no_faces());
  const float3 light_pos = mesh->compute_bbox().center();
  float3 normal = make_float3(0.0f);

  //dir = normalize(normal / no_of_faces);
  dir = normalize(light_pos - pos);
  const float dist = length(light_pos - pos);

  if (shadows) {
	  Ray& shadowRay = Ray(pos, dir, 0, EPSILON, dist-EPSILON);
	  HitInfo hit;

	  if (tracer->trace_to_any(shadowRay, hit)) {
		  return false;
	  }
  }

  unsigned int i = 0;
  for (; i < no_of_faces; i++) {
	  normal += normals.vertex(i);
  }
  normal = normalize(normal);
  for (i = 0; i < mesh->face_areas.size(); i++) {
	  L += get_emission(i) * mesh->face_areas.at(i) * abs(dot(dir, normals.vertex(i)) / (length(dir)*length(normals.vertex(i))));
  }
  L = L / (dist * dist);
  
  return true;
}

bool AreaLight::emit(Ray& r, HitInfo& hit, float3& Phi) const
{
  // Generate and trace a ray carrying radiance emitted from this area light.
  //
  // Output: r    (the new ray)
  //         hit  (info about the ray-surface intersection)
  //         Phi  (the flux carried by the emitted ray)
  //
  // Return: true if the ray hits anything when traced
  //
  // Relevant data fields that are available (see Light.h and Ray.h):
  // tracer              (pointer to ray tracer)
  // geometry            (indexed face set of triangle vertices)
  // normals             (indexed face set of vertex normals)
  // no_of_faces         (number of faces in triangle mesh for light source)
  // mesh->surface_area  (total surface area of the light source)
  // r.origin            (starting position of ray)
  // r.direction         (direction of ray)

  // Get geometry info
  const IndexedFaceSet& geometry = mesh->geometry;
	const IndexedFaceSet& normals = mesh->normals;
	const float no_of_faces = static_cast<float>(geometry.no_faces());

  // Sample ray origin and direction
 
  // Trace ray
  
  // If a surface was hit, compute Phi and return true

  return false;
}

float3 AreaLight::get_emission(unsigned int triangle_id) const
{
  const ObjMaterial& mat = mesh->materials[mesh->mat_idx[triangle_id]];
  return make_float3(mat.ambient[0], mat.ambient[1], mat.ambient[2]);
}
