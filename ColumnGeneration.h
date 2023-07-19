#pragma once

#include "ilcplex/ilocplex.h"
#include "ilconcert/cplexconcertdoc.h"

#include <vector>

class ColumnGeneration {
public:
	static void Build();

private:
	static std::vector<std::vector<int>> Q;
	
	static IloNumArray dual_variables;
	static double reduced_cost;

	static void initial_column(std::vector<int> item_length);
	static void MasterProblem(std::vector<int> item_length, std::vector<int> item_demand);
	static void SubProblems(std::vector<int> item_length);
	static void ShowCurrentSolution();
};