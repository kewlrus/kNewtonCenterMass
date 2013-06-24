#pragma once

#define ME_FIRST_MASS 200.0f

namespace nleStructures
{
	struct point
	{
		float x, y, r;
	};

	struct Combination
	{
		int SensorsNums[3];
	};

	struct SensorCombination
	{
		Combination* Combinations;
		int TotalCombinations;

		SensorCombination(int SensorCount, int* SensorsNums = NULL)
		{
			TotalCombinations = (int) ((SensorCount * (SensorCount - 1) * (SensorCount - 2)) / 6);
			Combinations = new Combination[TotalCombinations];
			int comb = 0;

			for (int i = 0; i < SensorCount; i++)
			{
				for (int j = i + 1; j < SensorCount ; j++)
				{
					for (int k = j + 1; k < SensorCount ; k++)
					{
						if (SensorsNums == NULL)
						{
							Combinations[comb].SensorsNums[0] = i;
							Combinations[comb].SensorsNums[1] = j;
							Combinations[comb].SensorsNums[2] = k;
						}
						else
						{
							Combinations[comb].SensorsNums[0] = SensorsNums[i];
							Combinations[comb].SensorsNums[1] = SensorsNums[j];
							Combinations[comb].SensorsNums[2] = SensorsNums[k];
						}
						comb++;
					}
				}
				if (comb >= TotalCombinations)
					break;
			}
		}

		~SensorCombination()
		{
			delete [] Combinations;
		}
	};

	struct SensorData
	{
		float X;
		float Y;
		float Distance;
	};

	struct ShootData
	{
		private:
			short int SensorsCount;

		public:
			SensorData* SD;
			int ScreenWidth;
			int ScreenHeight;

		public:
			void SetSensorData(SensorData* SD)
			{
				for (short int Sensor = 0; Sensor < SensorsCount; Sensor++)
				{
					this->SD[Sensor].X = SD[Sensor].X;
					this->SD[Sensor].Y = SD[Sensor].Y;
					this->SD[Sensor].Distance = SD[Sensor].Distance;
				}
			}

			short int GetSensorsCount()
			{
				return SensorsCount;
			}

			ShootData(short int SensorsCount)
			{
				this->SensorsCount = SensorsCount;
				this->SD = new SensorData[SensorsCount];
			}

			~ShootData()
			{
				delete [] SD;
			}
	};

	struct Result_items
	{
		point Coords;
		Combination SensorsTriggered;
	};

	struct Results
	{
		private:
			vector <nleStructures::Result_items>* ResultItems;

		public:
			vector<nleStructures::Result_items>* GetResults()
			{
				return ResultItems;
			}

			void AddResult(point* Coords, Combination* combination)
			{
				Result_items res;
				res.Coords.r = Coords ->r;
				res.Coords.x = Coords ->x;
				res.Coords.y = Coords ->y;

				res.SensorsTriggered.SensorsNums[0] = combination->SensorsNums[0];
				res.SensorsTriggered.SensorsNums[1] = combination->SensorsNums[1];
				res.SensorsTriggered.SensorsNums[2] = combination->SensorsNums[2];

				ResultItems->push_back(res);
			};


			Results()
			{
				ResultItems = new vector<nleStructures::Result_items>();
			};

			~Results()
			{
				
			};
	};
}

class nleNewtonSolver
{
public:
	nleStructures::point* GetPoint(nleStructures::SensorData* SensorData, short int SensorsCount, int Width, int Height);

	void AddXYR(float x, float y, float r_input);
	void ClearXY();
	nleStructures::point* FPiCalc(nleStructures::point* pi, bool modify, float post_dmg_area, float mdx);
	
	vector<nleStructures::Result_items>* GetResults(void) {return Results.GetResults(); };

	nleNewtonSolver(short int centers_count);
	virtual ~nleNewtonSolver(void);

private:
	nleStructures::Results Results;

	bool	FirstCompute;
	bool	LastCompute;
	float	FirstX, FirstY, FirstR;
	float	MassSum;
	float	MassComputeXChisl;
	float	MassComputeYChisl;

	boost::shared_array<float> x0, y0, z0, r, fpi;

	boost::shared_array<boost::shared_array<float>> wpi;
	boost::shared_array<boost::shared_array<float>> w_1pi;
	boost::shared_array<boost::shared_array<float>> ADop;

	float* Mul;

	nleStructures::point* old_pi;
	nleStructures::point* buff_pi;

	short int real_centers_count;
	short int centers_count;
	
	void WPiCalc(nleStructures::point* pi);
	void W_1PiCalc(boost::shared_array<boost::shared_array<float>> wpi, float det);
	void MulMatrix(boost::shared_array<boost::shared_array<float>> w_1pi, boost::shared_array<float> fpi, nleStructures::point* pi);
	float determinant(boost::shared_array<boost::shared_array<float>> wpi, short int razm);
	float function(float x, float x0, float y, float y0, float r);

	float df0dx(float x, float x0);
	float df0dy(float y, float y0);
	float df0dz(float z, float z0);
	float df0dr(float r);
	
	float dfdx(float x, float x0);
	float dfdy(float y, float y0);
	float dfdz(float z, float z0);
	float dfdr(float r, float r0);
};
