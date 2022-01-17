//Created: 15th Jan 2017
//Author: Cornelius Marx

#ifndef ZER0_VECTOR2D_H
#define ZER0_VECTOR2D_H

#define _USE_MATH_DEFINES
#include <math.h>

class zVector2D
{
public:

	//constructors
	zVector2D() : x(0.f), y(0.f){}
	zVector2D(float _x, float _y) : x(_x), y(_y){}
	zVector2D(const float * v) : x(v[0]), y(v[1]){}
	zVector2D(const zVector2D & v) : x(v.x), y(v.y){}

	//destructor
	~zVector2D(){}

	//initializers
	void loadZero(){x = 0.f; y = 0.f;}
	void loadOne(){x = 1.f; y = 1.f;}
	void set(float new_x, float new_y){x = new_x; y = new_y;}

	//calculations
	void normalize();
	zVector2D getNormalized() const;

	float getLength() const
	{return sqrt(x*x + y*y);}

	float getSquaredLength() const
	{return x*x + y*y;}

	void rotate(float rad); //rotates the vector about a given angle (rad)
	zVector2D getRotated(float rad) const;
	float getRotation() const; //returns the angle of vector to x-axis (rad)

	static float dot(const zVector2D & a, const zVector2D & b){return a.x*b.x + a.y*b.y;}
	static float cross(const zVector2D & a, const zVector2D & b){return a.x * b.y - b.x * a.y;}
	/* angle(rad) between two vectors */
	static float angle(const zVector2D & a, const zVector2D & b){return acos(dot(a.getNormalized(), b.getNormalized()));}
	static float signedAngle(const zVector2D & a, const zVector2D & b){return cross(a,b) < 0.0f ? -angle(a,b) : angle(a,b);}

	//overloaded operators
	zVector2D operator+(const zVector2D & v) const
	{return zVector2D(x + v.x, y + v.y);	}

	zVector2D operator-(const zVector2D & v) const
	{return zVector2D(x - v.x, y - v.y);	}

	zVector2D operator*(const float s) const
	{return zVector2D(x*s, y*s);	}
	
	zVector2D operator*(const zVector2D & v) const
	{return zVector2D(x*v.x, y*v.y);}
	
	zVector2D operator/(const float s) const
	{return zVector2D(x / s, y / s); }

	//allow operations like: 3*v
	friend zVector2D operator*(float scale, const zVector2D & v)
	{return zVector2D(v.x * scale, v.y * scale);}

	bool operator==(const zVector2D & v) const
	{return (x == v.x && y == v.y);}

	bool operator!=(const zVector2D & v) const
	{return (x != v.x || y != v.y);}

	void operator+=(const zVector2D & v)
	{x+=v.x;y+=v.y;}

	void operator-=(const zVector2D & v)
	{x-=v.x;y-=v.y;}

	void operator*=(const float s)
	{x*=s;y*=s;	}

	void operator/=(const float s)
	{x/=s; y/=s;}

	zVector2D operator-() const {return zVector2D(-x, -y);}
	zVector2D operator+() const {return *this;}

	//cast to pointer
	operator float* () const {return (float*) this;}

	//members
	float x, y;
};

#endif
