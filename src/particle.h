#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

#include "particlesystem.h"

class Particle : public ParticleSystem
{
public:
    Particle(Vector3f vel, Vector3f pos, float h);
    // evalF is a method defined in the ParticleSystem
    // interface. The use of virtual functions allows timesteppers to work
    // with any particle system (simple, pendulum, cloth), without
    // knowing which particular system it is.
    // Each ParticleSystem subclass must provide an implementation of evalF.
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    void updateVelocity(Vector3f vel);
    // void setState(std::vecstor<Vector3f> &state);
    // this is called from main.cpp when it's time to draw a new frame.
    // void draw(GLProgram&, Vector3f pos);

    private:
    float t,_h;
};

#endif
