#include "ColumnGeneration.h"


std::vector<std::vector<int>> ColumnGeneration::Q;
IloNumArray ColumnGeneration::dual_variables;
double ColumnGeneration::reduced_cost;
const int ColumnGeneration::NUM_ITEMS;
const int ColumnGeneration::MAX_LENGTH;

void ColumnGeneration::initial_column(std::vector<int> item_length) {
	ColumnGeneration::reduced_cost = 0;

	for (int i = 0; i < ColumnGeneration::NUM_ITEMS; ++i) {
		double number_piece = floor(ColumnGeneration::MAX_LENGTH / item_length[i]);
		
		std::vector<int> q;
		for (int j = 0; j < ColumnGeneration::NUM_ITEMS; ++j) {
			q.push_back(0);
		}
		q[i] = number_piece;
		ColumnGeneration::Q.push_back(q);
	}
}

void ColumnGeneration::Build() {

#pragma region Cutting Stock Problem solved by Column Generation

	std::vector<int> item_length = { 20, 45, 50, 55, 75 };
	std::vector<int> item_demand = { 48, 35, 24, 10, 8 };

	initial_column(item_length);

	do
	{
		ColumnGeneration::MasterProblem(item_length, item_demand);
		ColumnGeneration::SubProblems(item_length);
		ColumnGeneration::ShowCurrentSolution();
	} while (ColumnGeneration::reduced_cost < 0);

	std::cout << "Finish Column Generation Algorithm" << std::endl;

#pragma endregion

}

/*
Cutting Stock Problem
Restricted Master Problem
*/
void ColumnGeneration::MasterProblem(std::vector<int> item_length, std::vector<int> item_demand) {
	IloEnv env;
	IloModel model(env);
	IloCplex cplex(env);

	//Create decision variables 
	IloNumVarArray x(env, ColumnGeneration::Q.size(), 0, INT_MAX);
	

	// Objective function
	model.add(IloMinimize(env, IloSum(x)));


	// Constraints
	IloRangeArray all_constraints(env);
	IloExpr constraint(env);
	for (int i = 0; i < ColumnGeneration::NUM_ITEMS; ++i) {
		constraint.clear();
		for (int j = 0; j < ColumnGeneration::Q.size(); ++j) {
			constraint += ColumnGeneration::Q[j][i] * x[j];
		}
		all_constraints.add(constraint >= item_demand[i]);
	}
	//constraint.end();
	model.add(all_constraints);

	cplex.extract(model);
	cplex.exportModel("CG_master.lp");
	cplex.setOut(env.getNullStream());
	cplex.setWarning(env.getNullStream());
	cplex.setError(env.getNullStream());

	if (cplex.solve()) {
		// Dual variables 
		IloNumArray dual_sln(env);
		cplex.getDuals(dual_sln, all_constraints);

		ColumnGeneration::dual_variables = dual_sln;

		/*for (int i = 0; i < ColumnGeneration::NUM_ITEMS; ++i) {
			std::cout << ColumnGeneration::dual_variables[i] << std::endl;
		}*/
	}

	model.end();
	cplex.end();
}

/*
Knapsack Problem
*/
void ColumnGeneration::SubProblems(std::vector<int> item_length) {
	IloEnv env;
	IloModel model(env);
	IloCplex cplex(env);

	// Create decision variables 
	IloIntVarArray y(env, ColumnGeneration::NUM_ITEMS, 0, INT_MAX);

	// Objective function
	model.add(IloMaximize(env, IloScalProd(ColumnGeneration::dual_variables, y)));

	// Feasibility constraints
	IloExpr constraint(env);
	for (int i = 0; i < ColumnGeneration::NUM_ITEMS; ++i) {
		constraint += item_length[i] * y[i];
	}
	model.add(constraint <= ColumnGeneration::MAX_LENGTH);
	constraint.end();

	cplex.extract(model);
	cplex.exportModel("CG_sub.lp");
	cplex.setOut(env.getNullStream()); 
	cplex.setWarning(env.getNullStream());
	cplex.setError(env.getNullStream());

	if (cplex.solve()) {
		// Update reduced cost
		ColumnGeneration::reduced_cost = 1 - cplex.getObjValue();

		if (ColumnGeneration::reduced_cost < 0) {
			std::vector<int> new_column;
			for (int i = 0; i < ColumnGeneration::NUM_ITEMS; ++i) {
				new_column.push_back(cplex.getValue(y[i]));
			}
			ColumnGeneration::Q.push_back(new_column);
		}
	}

	cplex.end();
	model.end();
	env.end();
}

void ColumnGeneration::ShowCurrentSolution() {
	
	std::cout << "\n\n";
	std::cout << "=============================================" << std::endl;
	for (int i = 0; i < ColumnGeneration::Q.size(); ++i) {
		std::cout << "Pattern[" << i << "] = [";
		for (int q : ColumnGeneration::Q[i]) {
			std::cout << q << " ";
		}
		std::cout << "]" << std::endl;
	}
	std::cout << "=============================================" << std::endl;
	std::cout << "\n\n";
}
