#include <iostream>
#include <vector>
#include <string>
#include <ilcplex/ilocplex.h>

using namespace std; 

typedef IloArray<IloNumVarArray> NumVar2D; // 2D array 
typedef IloArray<NumVar2D> NumVar3D;      // 3D array


int main() {
	// param
	int pg = 45;
	int pb = 80;

	IloEnv env;
	IloModel model(env);
	// decision var
	IloNumVarArray x(env, 2, 0, IloInfinity, ILOINT);

	// obj func
	model.add(IloMaximize(env, x[0] * pg + x[1] * pb));

	// cons
	model.add(x[0] * 5 + x[1] * 20 <= 400);
	model.add(x[0] * 10 + x[1] * 15 <= 450);

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

