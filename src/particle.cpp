#include "particle.h"
#include "timestepper.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

Particle::Particle(Vector3f pos, Vector3f vel, float h, int n)
{
    // TODO 3.2 initialize the fluid system
    m_vVecState.push_back(pos);
    m_vVecState.push_back(vel);
    _h = h;
}   

Vector3f g = Vector3f(0.0,-9.8,0.0);

std::vector<Vector3f> Particle::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    
    f.push_back(state[1]);
    f.push_back(g);

    return f;
}

void Particle::updateVelocity(Vector3f vel){
    m_vVecState[1] = vel;
}