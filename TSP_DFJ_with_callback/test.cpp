// tsp_dfj_callback.cpp - TSP with DFJ using CPLEX Callback
#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <queue>

using namespace std;

// Function to find subtours from solution
vector<vector<int>> findSubtours(const vector<vector<double>>& sol, int n) {
    vector<bool> visited(n, false);
    vector<vector<int>> subtours;

    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            vector<int> component;
            queue<int> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty()) {
                int u = q.front();
                q.pop();
                component.push_back(u);

                for (int v = 0; v < n; v++) {
                    if (!visited[v] && sol[u][v] == 1) {
                        visited[v] = true;
                        q.push(v);
                    }
                }
            }

            if (component.size() > 1) {
                subtours.push_back(component);
            }
        }
    }
    return subtours;
}

// Global variables for callback
static IloArray<IloBoolVarArray> g_x;
static int g_n;

// Lazy constraint callback
ILOLAZYCONSTRAINTCALLBACK0(lazyCallback) {
    try {
        // Get current integer solution values
        vector<vector<double>> sol(g_n, vector<double>(g_n, 0.0));

        for (int i = 0; i < g_n; i++) {
            for (int j = 0; j < g_n; j++) {
                if (i != j) {
                    sol[i][j] = getValue(g_x[i][j]);
                }
            }
        }

        // Find subtours in the integer solution
        auto subtours = findSubtours(sol, g_n);

        // Check if we have exactly one tour covering all cities
        bool hasValidTour = (subtours.size() == 1 && subtours[0].size() == g_n);

        if (!hasValidTour) {
            // Add lazy constraints for each subtour
            for (const auto& subtour : subtours) { 
                if ((int)subtour.size() >= 2 && (int)subtour.size() <= g_n / 2) {    // vì constraints cho S và V\S là tương đương nên chỉ thêm cho S là đủ 
                                                                                    // S là hợp các subtour thì kiểu gì V\S cũng là hợp các subtour (CM được)
                    IloExpr cut(getEnv());
                    for (int i : subtour) {
                        for (int j : subtour) {
                            if (i != j) {
                                cut += g_x[i][j];
                            }
                        }
                    }

                    // Add lazy constraint
                    add(cut <= (int)subtour.size() - 1);
                    cut.end();

                    cout << "Added lazy constraint for subtour: ";
                    for (int city : subtour) cout << city << " ";
                    cout << "(size: " << subtour.size() << ")" << endl;
                }
            }
        }
    }
    catch (IloException& e) {
        cout << "Exception in lazy callback: " << e << endl;
    }
}

int main() {
    IloEnv env;
    try {
        int n = 6;
        double dist[6][6] = {
            {0, 12, 7, 15, 9, 20},
            {5, 0, 18, 11, 14, 16},
            {9, 13, 0, 8, 12, 10},
            {19, 7, 6, 0, 9, 17},
            {8, 15, 11, 14, 0, 6},
            {10, 9, 20, 13, 7, 0}
        };

        IloModel model(env);

        // Decision variables
        IloArray<IloBoolVarArray> x(env, n);
        for (int i = 0; i < n; i++) {
            x[i] = IloBoolVarArray(env, n);
            x[i][i].setBounds(0, 0); // No self loops
        }

        // Set global variables for callback
        g_x = x;
        g_n = n;

        // Objective function
        IloExpr obj(env);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    obj += dist[i][j] * x[i][j];
                }
            }
        }
        model.add(IloMinimize(env, obj));
        obj.end();

        // Degree constraints
        for (int i = 0; i < n; i++) {
            IloExpr outgoing(env), incoming(env);
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    outgoing += x[i][j];
                    incoming += x[j][i];
                }
            }
            model.add(outgoing == 1);
            model.add(incoming == 1);
            outgoing.end();
            incoming.end();
        }

        // Create CPLEX solver
        IloCplex cplex(model);

        // Set parameters
        cplex.setParam(IloCplex::Param::MIP::Display, 2);
        cplex.setParam(IloCplex::Param::Threads, 1);

        // Register callbacks
        cplex.use(lazyCallback(env));       // Lazy constraints (required)

        cout << "Solving TSP with DFJ callback method..." << endl;

        // Solve
        if (cplex.solve()) {
            cout << "\nOptimal solution found!" << endl;
            cout << "Optimal cost: " << cplex.getObjValue() << endl;

            // Reconstruct tour
            vector<int> tour;
            int current = 0;
            vector<bool> visited(n, false);

            tour.push_back(current);
            visited[current] = true;

            for (int step = 0; step < n - 1; step++) {
                for (int j = 0; j < n; j++) {
                    if (!visited[j] && cplex.getValue(x[current][j]) == 1) {
                        current = j;
                        tour.push_back(current);
                        visited[current] = true;
                        break;
                    }
                }
            }

            cout << "Optimal tour: ";
            for (size_t i = 0; i < tour.size(); i++) {
                cout << tour[i];
                if (i < tour.size() - 1) cout << " -> ";
            }
            cout << " -> " << tour[0] << endl;

            // Verify cost
            double totalCost = 0;
            for (size_t i = 0; i < tour.size(); i++) {
                int from = tour[i];
                int to = tour[(i + 1) % tour.size()];
                totalCost += dist[from][to];
            }
            cout << "Verified cost: " << totalCost << endl;
        }
        else {
            cout << "No solution found!" << endl;
            cout << "Status: " << cplex.getStatus() << endl;
        }
    }
    catch (IloException& e) {
        cerr << "CPLEX exception: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception" << endl;
    }

    env.end();
    return 0;
}