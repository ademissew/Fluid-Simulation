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
	virtual void takeStep2(ParticleSystem* particleSystem, float stepSize,int n) = 0;
	

};

//IMPLEMENT YOUR TIMESTEPPERS

class ForwardEuler : public TimeStepper
{
	void takeStep(ParticleSystem* particleSystem, float stepSize) override;
	void takeStep2(ParticleSystem* particleSystem, float stepSize,int n) override;

};

class Trapezoidal : public TimeStepper
{
	void takeStep(ParticleSystem* particleSystem, float stepSize) override;
	void takeStep2(ParticleSystem* particleSystem, float stepSize,int n) override;

};

class RK4 : public TimeStepper
{
	void takeStep(ParticleSystem* particleSystem, float stepSize) override;
	void takeStep2(ParticleSystem* particleSystem, float stepSize,int n) override;

};

/////////////////////////
#endif
