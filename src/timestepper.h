#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "vecmath.h"
#include <vector>
#include "particlesystem.h"

class TimeStepper
{
public:
    virtual ~TimeStepper() {}
	virtual void takeStep(ParticleSystem* particleSystem, float stepSize) = 0;
	virtual void takeStep2(ParticleSystem* particleSystem, float stepSize,int n, std::vector<std::vector<std::vector<float>>> grid_x, 
std::vector<std::vector<std::vector<float>>> grid_y,std::vector<std::vector<std::vector<float>>> grid_z) = 0;
	

};

//IMPLEMENT YOUR TIMESTEPPERS

class RK4 : public TimeStepper
{
	void takeStep(ParticleSystem* particleSystem, float stepSize) override;
	void takeStep2(ParticleSystem* particleSystem, float stepSize,int n, std::vector<std::vector<std::vector<float>>> grid_x, 
std::vector<std::vector<std::vector<float>>> grid_y,std::vector<std::vector<std::vector<float>>> grid_z) override;

};

/////////////////////////
#endif
