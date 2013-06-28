#ifndef _NLESOLVER_INTERFACE_
#define _NLESOLVER_INTERFACE_

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

class nleNewtonSolverInterface
{
public:
	virtual nleStructures::point* GetPoint(nleStructures::SensorData* SensorData, short int SensorsCount, int Width, int Height) = 0;

	virtual void AddXYR(float x, float y, float r_input) = 0;
	virtual void ClearXY() = 0;
	virtual nleStructures::point* FPiCalc(nleStructures::point* pi, bool modify, float post_dmg_area, float mdx) = 0;
	
	virtual vector<nleStructures::Result_items>* GetResults(void) = 0;

	nleNewtonSolverInterface(void){};
	virtual ~nleNewtonSolverInterface(void){};
};

extern nleNewtonSolverInterface* _stdcall CreateNleSolver(short int centers_count);
extern void _stdcall DeleteNleSolver(nleNewtonSolverInterface* SAG);

#endif