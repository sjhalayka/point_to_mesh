#ifndef main_H
#define main_H

#include "uv_camera.h"

#include "marching_cubes.h"




#include <cstdlib>
#include <GL/glut.h>       //GLUT Library

#include <iostream>
using std::cout;
using std::endl;

#include <iomanip>
using std::setprecision;

#include <vector>
using std::vector;

#include <string>
using std::string;

#include <sstream>
using std::ostringstream;
using std::istringstream;

#include <fstream>
using std::ofstream;
using std::ifstream;

#include <set>
using std::set;

#include <map>
using std::map;

#include <utility>
using std::pair;

#include <ios>
using std::ios_base;


void idle_func(void);
void init_opengl(const int& width, const int& height);
void reshape_func(int width, int height);
void display_func(void);
void keyboard_func(unsigned char key, int x, int y);
void mouse_func(int button, int state, int x, int y);
void motion_func(int x, int y);
void passive_motion_func(int x, int y);

void render_string(int x, const int y, void* font, const string& text);
void draw_objects(void);


vector<vertex_3> all_points;


vertex_3 background_colour(1.0f, 1.0f, 1.0f);
vertex_3 control_list_colour(0.0f, 0.0f, 0.0f);

bool draw_axis = true;
bool draw_control_list = true;

uv_camera main_camera;

GLint win_id = 0;
GLint win_x = 800, win_y = 600;
float camera_w = 20;

float camera_fov = 45;
float camera_x_transform = 0;
float camera_y_transform = 0;
float u_spacer = 0.01;
float v_spacer = 0.5 * u_spacer;
float w_spacer = 0.1;
float camera_near = 0.1f;
float camera_far = 10000.0f;

bool lmb_down = false;
bool mmb_down = false;
bool rmb_down = false;
int mouse_x = 0;
int mouse_y = 0;


void add_field_border(vector<float>& values, const float border_value, float& grid_min, float& grid_max, size_t& res)
{
	const float step_size = (grid_max - grid_min) / (res - 1);
	const float new_grid_min = grid_min - step_size;
	const float new_grid_max = grid_max + step_size;
	const size_t new_res = res + 2;

	vector<float> new_values(new_res * new_res * new_res, border_value);

	// Stuff new vector with old vector values
	for (size_t x = 0; x < res; x++)
		for (size_t y = 0; y < res; y++)
			for (size_t z = 0; z < res; z++)
				new_values[(x + 1) * new_res * new_res + (y + 1) * new_res + (z + 1)] = values[x * res * res + y * res + z];

	values = new_values;
	grid_min = new_grid_min;
	grid_max = new_grid_max;
	res = new_res;
}

bool write_triangles_to_binary_stereo_lithography_file(const vector<triangle>& triangles, const char* const file_name)
{
	cout << "Triangle count: " << triangles.size() << endl;

	if (0 == triangles.size())
		return false;

	// Write to file.
	ofstream out(file_name, ios_base::binary);

	if (out.fail())
		return false;

	const size_t header_size = 80;
	vector<char> buffer(header_size, 0);
	const unsigned int num_triangles = static_cast<unsigned int>(triangles.size()); // Must be 4-byte unsigned int.
	vertex_3 normal;

	// Write blank header.
	out.write(reinterpret_cast<const char*>(&(buffer[0])), header_size);

	// Write number of triangles.
	out.write(reinterpret_cast<const char*>(&num_triangles), sizeof(unsigned int));

	// Copy everything to a single buffer.
	// We do this here because calling ofstream::write() only once PER MESH is going to 
	// send the data to disk faster than if we were to instead call ofstream::write()
	// thirteen times PER TRIANGLE.
	// Of course, the trade-off is that we are using 2x the RAM than what's absolutely required,
	// but the trade-off is often very much worth it (especially so for meshes with millions of triangles).
	cout << "Generating normal/vertex/attribute buffer" << endl;

	// Enough bytes for twelve 4-byte floats plus one 2-byte integer, per triangle.
	const size_t data_size = (12 * sizeof(float) + sizeof(short unsigned int)) * num_triangles;
	buffer.resize(data_size, 0);

	// Use a pointer to assist with the copying.
	// Should probably use std::copy() instead, but memcpy() does the trick, so whatever...
	char* cp = &buffer[0];

	for (vector<triangle>::const_iterator i = triangles.begin(); i != triangles.end(); i++)
	{
		// Get face normal.
		vertex_3 v0 = i->vertex[1] - i->vertex[0];
		vertex_3 v1 = i->vertex[2] - i->vertex[0];

		normal = v0.cross(v1);
		normal.normalize();

		memcpy(cp, &normal.x, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &normal.y, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &normal.z, sizeof(float)); cp += sizeof(float);

		memcpy(cp, &i->vertex[2].x, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[2].y, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[2].z, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[1].x, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[1].y, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[1].z, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[0].x, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[0].y, sizeof(float)); cp += sizeof(float);
		memcpy(cp, &i->vertex[0].z, sizeof(float)); cp += sizeof(float);

		cp += sizeof(short unsigned int);
	}

	cout << "Writing " << data_size / 1048576.0f << " MB of data to binary Stereo Lithography file: " << file_name << endl;

	out.write(reinterpret_cast<const char*>(&buffer[0]), data_size);
	out.close();

	return true;
}



void tesselate_field(const vector<float>& values, vector<triangle>& triangle_list, const double& isovalue, const double& grid_min, const double& grid_max, const size_t& res)
{

	triangle_list.clear();

	const float step_size = (grid_max - grid_min) / (res - 1);

	for (size_t x = 0; x < res - 1; x++)
	{
		for (size_t y = 0; y < res - 1; y++)
		{
			for (size_t z = 0; z < res - 1; z++)
			{
				grid_cube temp_cube;

				size_t x_offset = 0;
				size_t y_offset = 0;
				size_t z_offset = 0;

				// Setup vertex 0
				x_offset = 0;
				y_offset = 0;
				z_offset = 0;
				temp_cube.vertex[0].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[0].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[0].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[0] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 1
				x_offset = 1;
				y_offset = 0;
				z_offset = 0;
				temp_cube.vertex[1].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[1].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[1].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[1] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 2
				x_offset = 1;
				y_offset = 0;
				z_offset = 1;
				temp_cube.vertex[2].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[2].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[2].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[2] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 3
				x_offset = 0;
				y_offset = 0;
				z_offset = 1;
				temp_cube.vertex[3].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[3].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[3].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[3] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 4
				x_offset = 0;
				y_offset = 1;
				z_offset = 0;
				temp_cube.vertex[4].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[4].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[4].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[4] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 5
				x_offset = 1;
				y_offset = 1;
				z_offset = 0;
				temp_cube.vertex[5].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[5].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[5].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[5] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 6
				x_offset = 1;
				y_offset = 1;
				z_offset = 1;
				temp_cube.vertex[6].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[6].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[6].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[6] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Setup vertex 7
				x_offset = 0;
				y_offset = 1;
				z_offset = 1;
				temp_cube.vertex[7].x = grid_min + ((x + x_offset) * step_size);
				temp_cube.vertex[7].y = grid_min + ((y + y_offset) * step_size);
				temp_cube.vertex[7].z = grid_min + ((z + z_offset) * step_size);
				temp_cube.value[7] = values[(x + x_offset) * (res) * (res)+(y + y_offset) * (res)+(z + z_offset)];

				// Generate triangles from cube
				triangle temp_triangle_array[5];

				short unsigned int number_of_triangles_generated = tesselate_grid_cube(isovalue, temp_cube, temp_triangle_array);

				for (short unsigned int i = 0; i < number_of_triangles_generated; i++)
				{
					triangle_list.push_back(temp_triangle_array[i]);
				}
			}
		}
	}
}


void convert_point_cloud_to_mesh(const vector<vertex_3>& points, size_t res, const char* const filename)
{
	vector<float> field;
	field.resize(res * res * res, 0.0);

	float curr_x_min = numeric_limits<float>::max();
	float curr_y_min = numeric_limits<float>::max();
	float curr_z_min = numeric_limits<float>::max();
	float curr_x_max = numeric_limits<float>::min();
	float curr_y_max = numeric_limits<float>::min();
	float curr_z_max = numeric_limits<float>::min();

	for (size_t i = 0; i < points.size(); i++)
	{


		if (points[i].x < curr_x_min)
			curr_x_min = points[i].x;

		if (points[i].x > curr_x_max)
			curr_x_max = points[i].x;

		if (points[i].y < curr_y_min)
			curr_y_min = points[i].y;

		if (points[i].y > curr_y_max)
			curr_y_max = points[i].y;

		if (points[i].z < curr_z_min)
			curr_z_min = points[i].z;

		if (points[i].z > curr_z_max)
			curr_z_max = points[i].z;
	}

	float x_extent = curr_x_max - curr_x_min;
	float y_extent = curr_y_max - curr_y_min;
	float z_extent = curr_z_max - curr_z_min;

	for (size_t i = 0; i < points.size(); i++)
	{

		float x_location = points[i].x - curr_x_min;
		size_t x_index = static_cast<size_t>(static_cast<double>(res)* (x_location / x_extent));

		if (x_index >= res)
			x_index = res - 1;

		float y_location = points[i].y - curr_y_min;
		size_t y_index = static_cast<size_t>(static_cast<double>(res) * (y_location / y_extent));

		if (y_index >= res)
			y_index = res - 1;

		float z_location = points[i].z - curr_z_min;
		size_t z_index = static_cast<size_t>(static_cast<double>(res) * (z_location / z_extent));

		if (z_index >= res)
			z_index = res - 1;

		size_t index = z_index * res * res;
		index += y_index * res;
		index += x_index;

		field[index] += 1;

		if (field[index] > 50)
			field[index] = 50;
	}

	add_field_border(field, 0.0f, curr_x_min, curr_x_max, res);


	vector<triangle> triangles;

	float field_max = numeric_limits<float>::min();
	float field_min = numeric_limits<float>::max();

	for (size_t i = 0; i < field.size(); i++)
	{
		if (field[i] < field_min)
			field_min = field[i];

		if (field[i] > field_max)
			field_max = field[i];
	}

	for (size_t i = 0; i < field.size(); i++)
	{
		field[i] = field[i] - field_min;
	}

	field_max = numeric_limits<float>::min();
	field_min = numeric_limits<float>::max();

	for (size_t i = 0; i < field.size(); i++)
	{
		if (field[i] < field_min)
			field_min = field[i];

		if (field[i] > field_max)
			field_max = field[i];
	}

	for (size_t i = 0; i < field.size(); i++)
		field[i] /= field_max;

	tesselate_field(field, triangles, 0.5f, curr_x_min, curr_x_max, res);

	write_triangles_to_binary_stereo_lithography_file(triangles, filename);
}





#endif
