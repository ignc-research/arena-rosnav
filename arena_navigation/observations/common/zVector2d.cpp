#include "zVector2d.h"

void zVector2D::normalize()
{
	*this = getNormalized();
}

zVector2D zVector2D::getNormalized() const
{
	return *this/getLength();
}

void zVector2D::rotate(float rad)
{
	*this = getRotated(rad);
}

zVector2D zVector2D::getRotated(float rad) const
{
	float sin_rad = sin(rad);
	float cos_rad = cos(rad);
	return zVector2D(x*cos_rad - y*sin_rad, x*sin_rad + y*cos_rad);
}

float zVector2D::getRotation() const
{
	if(x == 0.f){
		if(y < 0.0f)
		{
			return M_PI*1.5;
		}
		else{
			return M_PI*0.5;
		}
	}
	else if(y == 0.f){
		if(x < 0.0f)
		{
			return M_PI;
		}
		else{
			return 0.0;
		}
	}
	else{
		if(x > 0 && y > 0)
		{
			return atan(y/x);
		}
		else if(x < 0 && y > 0){
			return -atan(x/y) + M_PI*0.5;
		}
		else if(x < 0 && y < 0){
			return atan(y/x) + M_PI;
		}
		else if(x > 0 && y < 0){
			return -atan(x/y) + M_PI*1.5;
		}
	}
	return 0.f;
}
