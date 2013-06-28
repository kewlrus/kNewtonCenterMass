#pragma once

#define ME_FIRST_MASS 200.0f

class nleNewtonSolver : public nleNewtonSolverInterface
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
