#ifndef CELL_H
#define CELL_H

#include <vector>
#include "particle.h"

// #include "particlesystem.h"

class Cell
{
public:
    Cell(Vector3f pos, int _n);
    // evalF is a method defined in the ParticleSystem
    // interface. The use of virtual functions allows timesteppers to work
    // with any particle system (simple, pendulum, cloth), without
    // knowing which particular system it is.
    // Each ParticleSystem subclass must provide an implementation of evalF.
    // std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void fill(Particle particle);
    void unfill();

    // void updateVelocity(Vector3f vel);
    // void setState(std::vector<Vector3f> &state);
    // this is called from main.cpp when it's time to draw a new frame.
    void draw(GLProgram&);

    // std::vector<Vector3f> next_state;
    Particle _particle = Particle(Vector3f(0,0,0),Vector3f(0,0,0),0,1); //empty particle
    bool _filled;

    private:
    Vector3f _pos;
    int _n;
    // Particle _particle;
    // float t,_h;
};

#endif
