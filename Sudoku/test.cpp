#include <iostream>
#include <vector>
#include <string>
#include <ilcplex/ilocplex.h>

using namespace std; 

typedef IloArray<IloNumVarArray> NumVar2D; // 2D array 
typedef IloArray<NumVar2D> NumVar3D;      // 3D array


int main() {
	int sudoku[9][9] = {
	{5, 3, 0, 0, 7, 0, 0, 0, 0},
	{6, 0, 0, 1, 9, 5, 0, 0, 0},
	{0, 9, 8, 0, 0, 0, 0, 6, 0},
	{8, 0, 0, 0, 6, 0, 0, 0, 3},
	{4, 0, 0, 8, 0, 3, 0, 0, 1},
	{7, 0, 0, 0, 2, 0, 0, 0, 6},
	{0, 6, 0, 0, 0, 0, 2, 8, 0},
	{0, 0, 0, 4, 1, 9, 0, 0, 5},
	{0, 0, 0, 0, 8, 0, 0, 7, 9}
	};

	// 1. decision vars
	IloEnv env;
	IloModel model(env);
	NumVar3D x(env, 9);      //Xr,c,d = 1 if cell(r,c) = d, 
							//        = 0 (otherwise)

	for (int i = 0; i < 9; i++) 
	{
		x[i] = NumVar2D(env, 9);
		for (int j = 0; j < 9; j++)
		{
			x[i][j] = IloNumVarArray(env, 9, 0, 1, ILOINT);
		}
	}

	// 2. obj func
	// sudoku doesn't have any obj func => fix min c
	model.add(IloMinimize(env, 24));

	// 3. constraints 


	// 3.a. fix values given
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9; j++) {
			for (int k = 0; k < 9; k++) {
				if (sudoku[i][j] == k + 1)
				{
					model.add(x[i][j][k] == 1);
				}
			}
		}
	}


	// 3a. constraints of d
	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++) 
		{
			IloExpr consOfD(env);
			for (int k = 0; k < 9; k++)
			{
				consOfD += x[i][j][k];
			}
			model.add(consOfD == 1);
			consOfD.end();
		}
	}

	// 3b. 
	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			IloExpr consOfC(env);
			for (int k = 0; k < 9; k++)
			{
				consOfC += x[i][k][j];
			}
			model.add(consOfC == 1);
			consOfC.end();
		}
	}

	// 3c
	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			IloExpr consOfR(env);
			for (int k = 0; k < 9; k++)
			{
				consOfR += x[k][i][j];
			}
			model.add(consOfR == 1);
			consOfR.end();
		}
	}

	// 3d. block 3x3
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
		{
			for (int d = 0; d < 9; d++)
			{
				IloExpr consOfBlock(env);
				for (int p = i * 3; p <= i * 3 + 2; p++)
				{
					for (int q = j * 3; q <= j * 3 + 2; q++)
					{
						consOfBlock += x[p][q][d];
					}
				}
				model.add(consOfBlock == 1);
				consOfBlock.end();
			}
		}
	}
	

	IloCplex cplex(model);
	cplex.setOut(env.getNullStream());    // to clear cmd output
	if (!cplex.solve())     // timeout or out of memory,...
	{
		env.error() << "Failed to opt the problem" << endl;
		throw(-1);
	}

	// print results
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9; j++) {
			for (int k = 0; k < 9; k++) {
				if (cplex.getValue(x[i][j][k]) == 1)
				{
					sudoku[i][j] = k + 1;
				}
			}
		}
	}

	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 9; j++)
		{
			cout << sudoku[i][j] << " ";
		}
		cout << endl;
	}

	return 0;
}

