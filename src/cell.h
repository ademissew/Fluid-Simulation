#ifndef CELL_H
#define CELL_H

#include <vector>

#include "particlesystem.h"

class Cell : public ParticleSystem
{
public:
    Cell(Vector3f vel, Vector3f pos, float h);
    // evalF is a method defined in the ParticleSystem
    // interface. The use of virtual functions allows timesteppers to work
    // with any particle system (simple, pendulum, cloth), without
    // knowing which particular system it is.
    // Each ParticleSystem subclass must provide an implementation of evalF.
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void fill();
    void unfill();
    void setState(std::vector<Vector3f> &state);
    // this is called from main.cpp when it's time to draw a new frame.
    void draw(GLProgram&);

    std::vector<Vector3f> next_state;
    private:
    bool _filled;
    float _h;
};

#endif
