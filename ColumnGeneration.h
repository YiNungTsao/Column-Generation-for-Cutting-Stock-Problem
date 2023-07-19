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
	static const int NUM_ITEMS = 5;
	static const int MAX_LENGTH = 110;

	static void initial_column(std::vector<int> item_length);
	static void MasterProblem(std::vector<int> item_length, std::vector<int> item_demand);
	static void SubProblems(std::vector<int> item_length);
	static void ShowCurrentSolution();
};
