#pragma once
#include <math.h>

class Coordinate {
private:
	double x;
	double y;
public:
	double getX() {return this->x;}
	double getY() { return this->y; }
	void setX(double x) {this->x = x;}
	void setY(double y) {this->y = y;}
	static double EUC_2D(Coordinate a, Coordinate b)
	{
		return sqrt((a.getX() - b.getX()) * (a.getX() - b.getX()) + 
			(a.getY() - b.getY()) * (a.getY() - b.getY()));
	}

	Coordinate(double x, double y) {
		this->x = x;
		this->y = y;
	};

	Coordinate() {};
	~Coordinate() {};
};