// kNewtonCenterMass.cpp: определяет точку входа для консольного приложения.
//
#include "stdafx.h"

int _tmain(int argc, _TCHAR* argv[])
{
	short int SensorsCount = 4;

	nleStructures::SensorData* SensorData = new nleStructures::SensorData[SensorsCount];

	SensorData[0].Distance = 0.0f;
	SensorData[1].Distance = 593.0f * 0.125f * 0.404f; //20.0f;
	SensorData[2].Distance = 576.0f * 0.125f * 0.404f;
	SensorData[3].Distance = 425.0f * 0.125f * 0.404f;

	SensorData[0].X = 201.0f;
	SensorData[1].X = 1303.0f;
	SensorData[2].X = 1299.0f;
	SensorData[3].X = 192.0f;

	SensorData[0].Y = 651.0f;
	SensorData[1].Y = 647.0f;
	SensorData[2].Y = 103.0f;
	SensorData[3].Y = 107.0f;

	nleNewtonSolver NonLinerEquationsSolver(4);

	std::auto_ptr<nleStructures::point> res(NonLinerEquationsSolver.GetPoint(SensorData, SensorsCount, 1500, 750));
	printf("Result: (%0.2f, %0.2f)\n", res->x, res->y);
	vector <nleStructures::Result_items>* Results = NonLinerEquationsSolver.GetResults();

	for (vector<char*>::size_type i = 0; i < Results->size(); i++)
	{
		printf("%0.2f, %0.2f, %0.2f .. , %d, %d, %d\n"
		, Results->at(i).Coords.x
		, Results->at(i).Coords.y
		, Results->at(i).Coords.r
		, Results->at(i).SensorsTriggered.SensorsNums[0]
		, Results->at(i).SensorsTriggered.SensorsNums[1]
		, Results->at(i).SensorsTriggered.SensorsNums[2]
		);
	}
	
	SensorData[2].Y = 1000.0f;
	res.reset(NonLinerEquationsSolver.GetPoint(SensorData, SensorsCount, 2000, 1500));
	printf("Result: (%0.2f, %0.2f)\n", res->x, res->y);

	SensorData[2].Y = 1200.0f;
	res.reset(NonLinerEquationsSolver.GetPoint(SensorData, SensorsCount, 2000, 1500));
	printf("Result: (%0.2f, %0.2f)\n", res->x, res->y);

	SensorData[2].Y = 1400.0f;
	res.reset(NonLinerEquationsSolver.GetPoint(SensorData, SensorsCount, 2000, 1500));
	printf("Result: (%0.2f, %0.2f)\n", res->x, res->y);

	delete Results;

	_getch();

	return 0;
}

