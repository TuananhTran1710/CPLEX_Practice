// tsp_dfj_callback.cpp - TSP with DFJ using CPLEX Callback
#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <queue>

ILOSTLBEGIN

// Function to find subtours from solution
std::vector<std::vector<int>> findSubtours(const std::vector<std::vector<double>>& sol, int n) {
    std::vector<bool> visited(n, false);
    std::vector<std::vector<int>> subtours;

    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            std::vector<int> component;
            std::queue<int> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty()) {
                int u = q.front();
                q.pop();
                component.push_back(u);

                for (int v = 0; v < n; v++) {
                    if (!visited[v] && sol[u][v] > 0.5) {
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

// Lazy constraint callback using function-based approach
ILOUSERCUTCALLBACK0(subtourCallback) {
    try {
        // Get current solution values
        std::vector<std::vector<double>> sol(g_n, std::vector<double>(g_n, 0.0));

        for (int i = 0; i < g_n; i++) {
            for (int j = 0; j < g_n; j++) {
                if (i != j) {
                    sol[i][j] = getValue(g_x[i][j]);
                }
            }
        }

        // Find subtours
        auto subtours = findSubtours(sol, g_n);

        // Add cuts for each subtour that is not a complete tour
        for (const auto& subtour : subtours) {
            if ((int)subtour.size() < g_n && (int)subtour.size() >= 2) {

                IloExpr cut(getEnv());
                for (int i : subtour) {
                    for (int j : subtour) {
                        if (i != j) {
                            cut += g_x[i][j];
                        }
                    }
                }

                // Add the cut: sum of edges in subtour <= |subtour| - 1
                add(cut <= (int)subtour.size() - 1);
                cut.end();

                std::cout << "Added cut for subtour of size " << subtour.size() << std::endl;
            }
        }

    }
    catch (IloException& e) {
        std::cout << "Exception in callback: " << e << std::endl;
    }
}

// Lazy constraint callback
ILOLAZYCONSTRAINTCALLBACK0(lazyCallback) {
    try {
        // Get current integer solution values
        std::vector<std::vector<double>> sol(g_n, std::vector<double>(g_n, 0.0));

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
                if ((int)subtour.size() >= 2 && (int)subtour.size() < g_n) {

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

                    std::cout << "Added lazy constraint for subtour: ";
                    for (int city : subtour) std::cout << city << " ";
                    std::cout << "(size: " << subtour.size() << ")" << std::endl;
                }
            }
        }

    }
    catch (IloException& e) {
        std::cout << "Exception in lazy callback: " << e << std::endl;
    }
}

int main() {
    IloEnv env;
    try {
        int n = 6;
        double dist[6][6] = {
            {0, 10, 15, 20, 10, 12},
            {10, 0, 35, 25, 17, 18},
            {15, 35, 0, 30, 28, 24},
            {20, 25, 30, 0, 22, 26},
            {10, 17, 28, 22, 0, 14},
            {12, 18, 24, 26, 14, 0}
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
        //cplex.use(subtourCallback(env));      // User cuts (optional)
        cplex.use(lazyCallback(env));         // Lazy constraints (required)

        std::cout << "Solving TSP with DFJ callback method..." << std::endl;

        // Solve
        if (cplex.solve()) {
            std::cout << "\nOptimal solution found!" << std::endl;
            std::cout << "Optimal cost: " << cplex.getObjValue() << std::endl;

            // Reconstruct tour
            std::vector<int> tour;
            int current = 0;
            std::vector<bool> visited(n, false);

            tour.push_back(current);
            visited[current] = true;

            for (int step = 0; step < n - 1; step++) {
                for (int j = 0; j < n; j++) {
                    if (!visited[j] && cplex.getValue(x[current][j]) > 0.5) {
                        current = j;
                        tour.push_back(current);
                        visited[current] = true;
                        break;
                    }
                }
            }

            std::cout << "Optimal tour: ";
            for (size_t i = 0; i < tour.size(); i++) {
                std::cout << tour[i];
                if (i < tour.size() - 1) std::cout << " -> ";
            }
            std::cout << " -> " << tour[0] << std::endl;

            // Verify cost
            double totalCost = 0;
            for (size_t i = 0; i < tour.size(); i++) {
                int from = tour[i];
                int to = tour[(i + 1) % tour.size()];
                totalCost += dist[from][to];
            }
            std::cout << "Verified cost: " << totalCost << std::endl;

        }
        else {
            std::cout << "No solution found!" << std::endl;
            std::cout << "Status: " << cplex.getStatus() << std::endl;
        }

    }
    catch (IloException& e) {
        std::cerr << "CPLEX exception: " << e << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown exception" << std::endl;
    }

    env.end();
    return 0;
}