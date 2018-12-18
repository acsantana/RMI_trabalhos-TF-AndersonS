#ifndef _VECTORS_H
#define _VECTORS_H

#include <cmath>

typedef double VecType;

struct Vector2D{
    VecType x;
    VecType y;

    //multiply vector by scalar
    Vector2D operator*(const VecType& v) {
	Vector2D res;
	res.x = x*v;
	res.y = y*v;
        return res;
    }

    //sum vector by vector
    Vector2D operator+(const Vector2D& v){
	Vector2D res;
	res.x = x + v.x;
	res.y = y + v.y;
        return res;
    }

    //sub vector by vector
    Vector2D operator-(const Vector2D& v){
	Vector2D res;
	res.x = x - v.x;
	res.y = y - v.y;
        return res;
    }

    //divide vector by scalar
    Vector2D operator/(const VecType& v){
	Vector2D res;
	res.x = x/v;
	res.y = y/v;
        return res;
    }

    //vector length
    VecType operator!(){
        return sqrt(pow(x,2) + pow(y,2));
    }

    //unary vector
    Vector2D operator~(){
        return *this / !*this;
    }

};

#endif
