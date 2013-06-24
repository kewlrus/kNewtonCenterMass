#include "stdafx.h"

int SortCompare(const void* a, const void* b)
{
	return (int) ( ((nleStructures::SensorData*)a)->Distance - ((nleStructures::SensorData*)b)->Distance );
}

nleNewtonSolver::nleNewtonSolver(short int centers_count)
{
	this->centers_count = centers_count;
	x0 = boost::shared_array<float>(new float[centers_count]);
	y0 = boost::shared_array<float>(new float[centers_count]);
	z0 = boost::shared_array<float>(new float[centers_count]);
	r = boost::shared_array<float>(new float[centers_count]);

	fpi = boost::shared_array<float>(new float[centers_count]);

	wpi = boost::shared_array<boost::shared_array<float>>(new boost::shared_array<float>[centers_count]);
	for (int i = 0; i < centers_count; i++)
		wpi[i] = boost::shared_array<float>(new float[centers_count]);

	w_1pi = boost::shared_array<boost::shared_array<float>>(new boost::shared_array<float>[centers_count]);
	for (int i = 0; i < centers_count; i++)
		w_1pi[i] = boost::shared_array<float>(new float[centers_count]);

	real_centers_count = 0;

	Mul = NULL;

	old_pi = new nleStructures::point();
	buff_pi = new nleStructures::point();
}

nleNewtonSolver::~nleNewtonSolver(void)
{
	delete old_pi;
	delete buff_pi;
}

nleStructures::point* nleNewtonSolver::GetPoint(nleStructures::SensorData* SensorData, short int SensorsCount, int Width, int Height)
{
	nleStructures::point* p0 = new nleStructures::point();
	nleStructures::point* res = NULL;
	nleStructures::ShootData ShootData(SensorsCount);

	ShootData.ScreenHeight = Height;
	ShootData.ScreenWidth = Width;
	ShootData.SetSensorData(SensorData);

	qsort(ShootData.SD, ShootData.GetSensorsCount(), sizeof(nleStructures::SensorData), SortCompare);
	
	nleStructures::SensorCombination* SensorCombinations = new nleStructures::SensorCombination(ShootData.GetSensorsCount());
	
	LastCompute = false;
	FirstCompute = true;

	float x = 0.0f;
	float y = 0.0f;
	float r = 0.0f;

	for (int i = 0; i < SensorCombinations->TotalCombinations; i++)
	{
		if (i == SensorCombinations->TotalCombinations - 1)
			LastCompute = true;

		short int s0 = SensorCombinations->Combinations[i].SensorsNums[0];
		short int s1 = SensorCombinations->Combinations[i].SensorsNums[1];
		short int s2 = SensorCombinations->Combinations[i].SensorsNums[2];

		float delay_a = ShootData.SD[s1].Distance;
		float delay_c = ShootData.SD[s2].Distance;

		p0->r = 0.0f;
		float right_part = (sqrt(pow(ShootData.SD[s0].X - ShootData.SD[s1].X, 2.0f) + pow(ShootData.SD[s0].Y - ShootData.SD[s1].Y, 2)) - delay_a) / 2.0f;
		float right_part2 = (sqrt(pow(ShootData.SD[s0].X - ShootData.SD[s2].X, 2.0f) + pow(ShootData.SD[s0].Y - ShootData.SD[s2].Y, 2)) - delay_c) / 2.0f;

		if (p0->r < right_part)
			p0->r = right_part;

		if (p0->r < right_part2)
			p0->r = right_part2;

		if (p0->r <= 0.0f)
			p0->r = (float)ShootData.ScreenWidth / 2;

		p0->x = (ShootData.SD[s0].X + ShootData.SD[s1].X + ShootData.SD[s2].X) / (float)3.0;
		p0->y = (ShootData.SD[s0].Y + ShootData.SD[s1].Y + ShootData.SD[s2].Y) / (float)3.0;

		ClearXY();

		AddXYR(ShootData.SD[s0].X, ShootData.SD[s0].Y, 0);
		AddXYR(ShootData.SD[s1].X, ShootData.SD[s1].Y, ShootData.SD[s1].Distance);
		AddXYR(ShootData.SD[s2].X, ShootData.SD[s2].Y, ShootData.SD[s2].Distance);

		res = FPiCalc(p0, true, 0.6f, 0.0f);
		
		x = res->x;
		y = res->y;
		r = res->r;

		Results.AddResult(res, &SensorCombinations->Combinations[i]);

		if (FirstCompute) 
		{
			FirstCompute = false;
			FirstX = x;
			FirstY = y; 
			FirstR = r;
			MassSum = ME_FIRST_MASS;
			MassComputeXChisl = ME_FIRST_MASS * FirstX;
			MassComputeYChisl = ME_FIRST_MASS * FirstY;
		}
		else
		{
			float Distantion = sqrt(pow((float)(FirstX - x), 2.0f) + pow((float)(FirstY - y), 2.0f));

			if (Distantion <= 60.0f)
			{
				if (Distantion < 1.0f)
					Distantion = 1.0f;

				float Mass = 1200.0f / Distantion;

				if (Mass > ME_FIRST_MASS)
					Mass = ME_FIRST_MASS;

				MassSum += Mass;

				MassComputeXChisl += Mass * x;
				MassComputeYChisl += Mass * y;
			}

		}

		if (LastCompute)
		{
			x = MassComputeXChisl / MassSum;
			y = MassComputeYChisl / MassSum;
		}
	}

	res->x = x;
	res->y = y;

	delete SensorCombinations;

	return res;
}

float nleNewtonSolver::function(float x, float x0, float y, float y0, float r)
{
	return (x - x0) * (x - x0) + (y - y0) * (y - y0) - r * r;
}

void nleNewtonSolver::AddXYR(float x, float y, float r_input)
{
	x0[real_centers_count] = x;
	y0[real_centers_count] = y;
	r[real_centers_count] = r_input;
	real_centers_count++;
}

void nleNewtonSolver::ClearXY()
{
	real_centers_count = 0;
}
float nleNewtonSolver::df0dx(float x, float x0)
{
	return (2 * x - 2 * x0);
}
float nleNewtonSolver::df0dy(float y, float y0)
{
	return (2 * y - 2 * y0);
}
float nleNewtonSolver::df0dz(float z, float z0)
{
	return (2 * z - 2 * z0);
}
float nleNewtonSolver::df0dr(float r)
{
	return (-2 * r);
}

float nleNewtonSolver::dfdx(float x, float x0)
{
	return (2 * x - 2 * x0);
}
float nleNewtonSolver::dfdy(float y, float y0)
{
	return (2 * y - 2 * y0);
}
float nleNewtonSolver::dfdr(float r, float r0)
{
	return (-2 * r - 2 * r0);
}

nleStructures::point * nleNewtonSolver::FPiCalc(nleStructures::point *pi, bool modify, float post_dmg_area, float mdx)
{
	short int count = 0;
	do
	{
		count++;

		for (int i=0; i<real_centers_count; i++)
			fpi[i] = function(pi->x, x0[i], pi->y, y0[i], pi->r + r[i]);

		WPiCalc(pi);
		W_1PiCalc(wpi, determinant(wpi, real_centers_count));
		buff_pi->r = old_pi->r;
		buff_pi->x = old_pi->x;
		buff_pi->y = old_pi->y;

		old_pi->r = pi->r;
		old_pi->x = pi->x;
		old_pi->y = pi->y;
		MulMatrix(w_1pi, fpi, pi);
		if ((buff_pi->r == pi->r) && (buff_pi->x == pi->x) && (buff_pi->y == pi->y) || count > 200)
			break;
	}
	while (abs(old_pi->r - pi->r ) > 0.001 && abs(old_pi->x - pi->x) > 0.001 && abs(old_pi->y - pi->y) > 0.001
		|| (abs(old_pi->r - pi->r) + abs(old_pi->x - pi->x)+abs(old_pi->y - pi->y) > 0.1)
		);

	if (pi->x < 0.0)
		pi->x = 0.0;

	if (pi->y < 0.0)
		pi->y = 0.0;

	return pi;
}

void nleNewtonSolver::WPiCalc(nleStructures::point *pi)
{
	for (int i = 0; i<real_centers_count; i++)
	{
		for (int j = 0; j<real_centers_count; j++)
		{
			if (i == 0)
			{
				switch (j)
				{
					case 0:
						wpi[i][j] = df0dx(pi->x,x0[i]);
						break;
					case 1:
						wpi[i][j] = df0dy(pi->y,y0[i]);
						break;
					case 2:
						wpi[i][j] = df0dr(pi->r);
						break;
				}
			}
			else
			{
				switch (j)
				{
					case 0:
						wpi[i][j] = dfdx(pi->x,x0[i]);
						break;
					case 1:
						wpi[i][j] = dfdy(pi->y,y0[i]);
						break;
					case 2:
						wpi[i][j] = dfdr(pi->r,r[i]);
						break;
				}
			}
		}
	}
}

float nleNewtonSolver::determinant(boost::shared_array<boost::shared_array<float>> wpi, short int razm)
{
	float det  =  0.0f;
	if (razm == 3)
	{
		det = wpi[0][0] * wpi[1][1] * wpi[2][2]
		+wpi[0][1] * wpi[1][2] * wpi[2][0]
		+wpi[1][0] * wpi[2][1] * wpi[0][2]
		-wpi[2][0] * wpi[1][1] * wpi[0][2]
		-wpi[0][1] * wpi[1][0] * wpi[2][2]
		-wpi[0][0] * wpi[1][2] * wpi[2][1];
	}
	else 
	{
		if (razm == 2)
			det = wpi[0][0] * wpi[1][1]
			-wpi[0][1] * wpi[1][0];
		else
		{
			ADop = boost::shared_array<boost::shared_array<float>>(new boost::shared_array<float>[razm - 1]);
			for (int i = 0; i < centers_count; i++)
				ADop[i] = boost::shared_array<float>(new float[razm - 1]);

			int id, jd = 0;
			bool flag = false;
			for (int i = 0; i < razm; i++)
			{
				int j = 0;
				id = 0;
				for (int x = 0; x < razm; x++)
				{
					jd = 0;
					flag = false;
					for (int y = 0; y < razm; y++)
					{
						if ((x != i) && (y != j))
						{
							ADop[id][jd] = wpi[x][y];
							flag = true;
							jd++;
						}
					}
					if (flag)
						id++;
				}
				float mul = -1;
				for (int k = 0; k <= i+j; k++)
					 mul *= -1;

				det += mul * wpi[i][j] * determinant(ADop, razm - 1);
			}
		}
	}
	return det;
}


void nleNewtonSolver::W_1PiCalc(boost::shared_array<boost::shared_array<float>> wpi, float det)
{
	if (ADop == NULL)
	{
		ADop = boost::shared_array<boost::shared_array<float>>(new boost::shared_array<float>[real_centers_count - 1]);
		for (int i = 0; i < real_centers_count - 1; i++)
			ADop[i] = boost::shared_array<float>(new float[real_centers_count - 1]);
	}
		
	bool flag = false;
	int id, jd = 0;
	for (int i = 0; i < real_centers_count; i++)
	{
		for (int j = 0; j < real_centers_count; j++)
		{
			id = 0;
			for (int x = 0; x < real_centers_count; x++)
			{
				jd = 0;
				flag = false;
				for (int y = 0; y < real_centers_count; y++)
				{
					if ((x != i) && (y != j))
					{
						ADop[id][jd] = wpi[x][y];
						flag = true;
						jd++;
					}
				}
				if (flag)
					id++;
			}
			float mul = -1;
			for (int k = 0; k <= i + j; k++)
				 mul *= -1;

			w_1pi[j][i] = mul * determinant(ADop, real_centers_count - 1) / det;
		}
	}
}

void nleNewtonSolver::MulMatrix(boost::shared_array<boost::shared_array<float>> w_1pi, boost::shared_array<float> fpi, nleStructures::point* pi)
{
	if (Mul == NULL)
		Mul = new float[real_centers_count];

	for (int i = 0; i < real_centers_count; i++)
		Mul[i] = 0;

	for (int i = 0; i < real_centers_count; i++)
	{
		for (int j = 0; j < real_centers_count; j++)
			Mul[i] += w_1pi[i][j] * fpi[j];
		switch (i)
		{
			case 0:
				pi->x -= Mul[i];
				break;
			case 1:
				pi->y -= Mul[i];
				break;
			case 2:
				pi->r -= Mul[i];
				break;
		}
	}
}