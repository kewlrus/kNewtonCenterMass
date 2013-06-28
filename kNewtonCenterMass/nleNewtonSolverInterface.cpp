#include "StdAfx.h"

nleNewtonSolverInterface* _stdcall CreateNleSolver(short int centers_count)
{
	return (new nleNewtonSolver(centers_count));
}

void _stdcall DeleteNleSolver(nleNewtonSolverInterface* nleSolver)
{
	delete nleSolver;
}