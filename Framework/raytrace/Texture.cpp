// 02562 Rendering Framework
// Written by Jeppe Revall Frisvad, 2011
// Copyright (c) DTU Informatics 2011

#include <iostream>
#include <optix_world.h>
#include "my_glut.h"
#include "../SOIL/SOIL.h"
#include "Texture.h"

using namespace std;
using namespace optix;

void Texture::load(const char* filename)
{
	SOIL_free_image_data(data);
	data = SOIL_load_image(filename, &width, &height, &channels, SOIL_LOAD_AUTO);
	if (!data)
	{
		cerr << "Error: Could not load texture image file." << endl;
		return;
	}
	int img_size = width*height;
	delete[] fdata;
	fdata = new float4[img_size];
	for (int i = 0; i < img_size; ++i)
		fdata[i] = look_up(i);
	tex_handle = SOIL_create_OGL_texture(data, width, height, channels, tex_handle, SOIL_FLAG_INVERT_Y | SOIL_FLAG_TEXTURE_REPEATS);
	tex_target = GL_TEXTURE_2D;
}

void Texture::load(GLenum target, GLuint texture)
{
	glBindTexture(target, texture);
	glGetTexLevelParameteriv(target, 0, GL_TEXTURE_WIDTH, &width);
	glGetTexLevelParameteriv(target, 0, GL_TEXTURE_HEIGHT, &height);
	delete[] fdata;
	fdata = new float4[width*height];
	glGetTexImage(target, 0, GL_RGBA, GL_FLOAT, &fdata[0].x);
	tex_handle = texture;
	tex_target = target;
}

float4 Texture::sample_nearest(const float3& texcoord) const
{
	if (!fdata)
		return make_float4(0.0f);

	// Implement texture look-up of nearest texel.
	//
	// Input:  texcoord      (texture coordinates: u = texcoord.x, v = texcoord.y)
	//
	// Return: texel color found at the given texture coordinates
	//
	// Relevant data fields that are available (see Texture.h)
	// fdata                 (flat array of texture data: texel colors in float4 format)
	// width, height         (texture resolution)
	//
	// Hint: Remember to revert the vertical axis when finding the index
	//       into fdata.

	int u = (int)((texcoord.x - floor(texcoord.x))*(width - 1) + 0.5f) % width;
	int v = (int)((1 - texcoord.y + floor(texcoord.y))*(height - 1) + 0.5f) % height;

	return fdata[u + v*width];
}

float4 Texture::sample_linear(const float3& texcoord) const
{
	if (!fdata)
		return make_float4(0.0f);

	// Implement texture look-up which returns the bilinear interpolation of
	// the four nearest texel neighbors.
	//
	// Input:  texcoord      (texture coordinates: u = texcoord.x, v = texcoord.y)
	//
	// Return: texel color found at the given texture coordinates after
	//         bilinear interpolation
	//
	// Relevant data fields that are available (see Texture.h)
	// fdata                 (flat array of texture data: texel colors in float4 format)
	// width, height         (texture resolution)
	//
	// Hint: Use three lerp operations (or one bilerp) to perform the
	//       bilinear interpolation.

	float a, b;
	float2 zerozero, onezero, zeroone, oneone;
	float4 result1, result2;
	a = (texcoord.x - floor(texcoord.x)) * width;
	b = (texcoord.y - floor(texcoord.y)) * height;

	zerozero = make_float2(floor(a), floor(b));
	onezero = make_float2(fmodf(ceil(a), width), floor(b));
	zeroone = make_float2(floor(a), fmodf(ceil(b), height));
	oneone = make_float2(fmodf(ceil(a), width), fmodf(ceil(b), height));

	result1 = lerp(fdata[(int)zerozero.x + (int)zerozero.y * width], fdata[(int)onezero.x + (int)onezero.y * width], a - floor(a));
	result2 = lerp(fdata[(int)zeroone.x + (int)zeroone.y * width], fdata[(int)oneone.x + (int)oneone.y * width], a - floor(a));

	return lerp(result1, result2, b - floor(b));
}

float4 Texture::look_up(unsigned int idx) const
{
	idx *= channels;
	switch (channels)
	{
	case 1:
	{
		float v = convert(data[idx]);
		return make_float4(v, v, v, 1.0f);
	}
	case 2:
		return make_float4(convert(data[idx]), convert(data[idx]), convert(data[idx]), convert(data[idx + 1]));
	case 3:
		return make_float4(convert(data[idx]), convert(data[idx + 1]), convert(data[idx + 2]), 1.0f);
	case 4:
		return make_float4(convert(data[idx]), convert(data[idx + 1]), convert(data[idx + 2]), convert(data[idx + 3]));
	}
	return make_float4(0.0f);
}

float Texture::convert(unsigned char c) const
{
	return (c + 0.5f) / 256.0f;
}
