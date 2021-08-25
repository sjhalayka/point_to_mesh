#ifndef MARCHING_CUBES_H
#define MARCHING_CUBES_H

#include <cmath>
using std::fabs;

#include <iostream>
using std::cout;
using std::endl;

#include <vector>
using std::vector;


#include <limits>
using std::numeric_limits;


// Modified from Paul Bourke, Polygonising a Scalar Field
// http://local.wasp.uwa.edu.au/~pbourke/geometry/polygonise/index.html

class vertex_3
{
public:


	void rotate_x(const double& radians)
	{
		double t_y = y;

		y = t_y * cos(radians) + z * sin(radians);
		z = t_y * -sin(radians) + z * cos(radians);
	}

	void rotate_y(const double& radians)
	{
		double t_x = x;

		x = t_x * cos(radians) + z * -sin(radians);
		z = t_x * sin(radians) + z * cos(radians);
	}

	void zero(void)
	{
		x = y = z = 0;
	}

	vertex_3(float src_x, float src_y, float src_z)
	{
		x = src_x;
		y = src_y;
		z = src_z;
	}

	vertex_3(void)
	{
		x = y = z = 0;
	}

	float x;
	float y;
	float z;


	inline const void normalize(void)
	{
		float len = length();

		if (0.0f != len)
		{
			x /= len;
			y /= len;
			z /= len;
		}
	}

	inline bool operator==(const vertex_3 &right) const
	{
		if(right.x == x && right.y == y && right.z == z)
			return true;
		else
			return false;
	}

	inline bool operator<(const vertex_3 &right) const
	{
		if(right.x > x)
			return true;
		else if(right.x < x)
			return false;
			
		if(right.y > y)
			return true;
		else if(right.y < y)
			return false;

		if(right.z > z)
			return true;
		else if(right.z < z)
			return false;

		return false;
	}

	inline const vertex_3& operator-(const vertex_3 &right) const
	{
		static vertex_3 temp;

		temp.x = this->x - right.x;
		temp.y = this->y - right.y;
		temp.z = this->z - right.z;

		return temp;
	}

	inline const vertex_3& cross(const vertex_3 &right) const
	{
		static vertex_3 temp;

		temp.x = y*right.z - z*right.y;
		temp.y = z*right.x - x*right.z;
		temp.z = x*right.y - y*right.x;

		return temp;
	}

	inline double dot(const vertex_3 &right) const
	{
		return x*right.x + y*right.y + z*right.z;
	}

	inline const double self_dot(void)
	{
		return x*x + y*y + z*z;
	}

	inline const double length(void)
	{
		return sqrt(self_dot());
	}
};

class triangle
{
public:
	vertex_3 vertex[3];
	vertex_3 normal;

	inline double area(void)
	{
		// If degenerate triangles are produced, then the marching cubes epsilon variable may not be small enough in value (see: function marching_cubes.cpp::vertex_interp()).
		if(vertex[0] == vertex[1] || vertex[0] == vertex[2] || vertex[1] == vertex[2])
			return 0.0;

		static vertex_3 a, b, cross;

		// Same vertex winding order (surface normal direction) as OpenGL.
		a = vertex[0] - vertex[1];
		b = vertex[0] - vertex[2];

		cross = a.cross(b);

		return 0.5f*cross.length();
	}
};

class grid_cube
{
public:
	vertex_3 vertex[8];
	double value[8];
};

class marched_cube
{
public:
	vertex_3 centre_point;
	vector<triangle> triangles; // A small number of triangles.
};

vertex_3 vertex_interp(const double &isolevel, const vertex_3 p1, const vertex_3 p2, const double valp1, const double valp2);
short unsigned int tesselate_grid_cube(const double &isovalue, grid_cube grid, triangle *const triangles);


#endif
