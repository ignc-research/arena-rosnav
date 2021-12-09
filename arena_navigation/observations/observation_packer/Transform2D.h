#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <common/zVector2d.h>

struct Transform2D{
	zVector2D position;	
	float theta;
	Transform2D(): position(0,0), theta(0){}
	Transform2D(const Transform2D & t): position(t.position), theta(t.theta){}
	Transform2D(float x, float y, float _theta){
		position.x = x;
		position.y = y;
		theta = _theta;
	}
	Transform2D(const zVector2D & _position, float _theta): position(_position), theta(_theta){}

	void set(float x, float y, float _theta){
		position.set(x, y);
		theta = _theta;
	}

	void set(const zVector2D & pos, float _theta){
		position = pos;
		theta = _theta;
	}

	void setPosition(float x, float y){
		position.x = x;
		position.y = y;
	}

	void setPosition(const zVector2D & pos){
		position = pos;
	}

	void setTheta(float angle){
		theta = angle;	
	}
	void setFromMsg(const geometry_msgs::Pose & pose){
		position.x = pose.position.x;
		position.y = pose.position.y;
		tf2::Quaternion q(
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w);
		theta = q.getAngle()*q.getAxis().getZ();
	}
	void getDistance(const Transform2D & other, float & distance, float & angle)const{
		zVector2D v = other.position-position;
		distance = v.getLength();
		angle = zVector2D::signedAngle(v, zVector2D(cos(theta), sin(theta)));
	}

	// translate with respect to local coordinate system
	void localTranslate(const zVector2D & v){
		position += v.getRotated(theta);
	}

	Transform2D getLocalTranslate(const zVector2D & v)const{
		Transform2D t(*this);
		t.localTranslate(v);
		return t;
	}

	// change this transform to a relative transform with respect to t
	void toLocalOf(const Transform2D & t){
		position -= t.position;
		position.rotate(-t.theta);
		theta -= t.theta;
	}

	Transform2D getToLocalOf(const Transform2D & t)const{
		Transform2D new_t(*this);
		new_t.toLocalOf(t);
		return new_t;
	}

	void transform(const Transform2D & t){
		position += t.position;
		theta += t.theta;
	}
};

// void packMsg(observations::Observation & obs);
