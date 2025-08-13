#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <ilcplex\ilocplex.h>
#include <chrono>;
#include "Coord.h"
using namespace std;

typedef IloArray<IloNumVarArray> NumVar2D; 


int main(int argc, char* argv[])      
{
	// handle params
	/*int n;
	ifstream in("tsplib/a280.tsp");
	string line;
	vector<Coordinate> coords;*/

	//// read dimension
	//while (getline(in, line)) {
	//	if (line.find("DIMENSION") != string::npos) {
	//		string dummy;
	//		stringstream ss(line);
	//		ss >> dummy >> dummy >> n; // DIMENSION: n
	//	}
	//	if (line.find("NODE_COORD_SECTION") != string::npos) {
	//		break; // tới phần tọa độ
	//	}
	//}
	//
	//coords.resize(n);
	//for (int i = 0; i < n; i++) {
	//	int id;
	//	double x, y;
	//	in >> id >> x >> y;
	//	coords[i].setX(x);
	//	coords[i].setY(y);
	//}


	//int** c = new int* [n];

	//for (int i = 0; i < n; i++) {
	//	c[i] = new int[n];
	//	for (int j = 0; j < n; j++) {
	//		c[i][j] = Coordinate::EUC_2D(coords[i], coords[j]);
	//	}
	//}
	// 
	

	int c[3][3] = {
		{8, 5, 9 },
		{10, 12, 13 },
		{3,  4, 1 },
	};
	int n = 3;
	 //decision var
	IloEnv env;
	IloModel model(env);
	NumVar2D x(env, n);
	for (int i = 0; i < n; i++) 
	{
		x[i] = IloNumVarArray(env, n, 0, 1, ILOINT);
	}

	IloNumVarArray u(env, n, 1, n-1, ILOINT);      

	// obj func
	IloExpr objFunc(env);
	for (int i = 0; i < n; i++) 
	{
		for (int j = 0; j < n; j++) 
		{
			if (i != j)
			{
				objFunc += c[i][j] * x[i][j];
			}
		}
	}

	model.add(IloMinimize(env, objFunc));

	// constraints
	
	for (int i = 0; i < n; i++) 
	{
		IloExpr incomeCons(env);
		for (int j = 0; j < n; j++)
		{
			if (i != j)
			{
				incomeCons += x[j][i];
			}
		}
		model.add(incomeCons == 1);
	}


	for (int i = 0; i < n; i++)
	{
		IloExpr outgoingCons(env);
		for (int j = 0; j < n; j++)
		{
			if (i != j)
			{
				outgoingCons += x[i][j];
			}
		}
		model.add(outgoingCons == 1);
	}


	for (int i = 1; i < n; i++)
	{
		for (int j = 1; j < n; j++)
		{
			IloExpr cons(env);
			if (i != j)
			{	
				cons += u[i] - u[j] + (n - 1) * x[i][j];
				model.add(cons <= n - 2);
			}
		}
	}

	IloCplex cplex(model);

	//cplex.setOut(env.getNullStream());    // to clear cmd output
	if (!cplex.solve())     // timeout or out of memory,...
	{
		env.error() << "Failed to opt the problem" << endl;
		throw(-1);
	}

	//get results
	double obj = cplex.getObjValue();

	cout << "My optimal objective func is: " << obj << endl;

	return 0;
}

