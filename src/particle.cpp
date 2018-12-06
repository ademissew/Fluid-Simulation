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
    // Vector3f pos = state[0] + _h*state[1];
    // std::cout <<state[0][0] + _h*state[1][0] << " " <<state[0][1] + _h*state[1][1] << " " <<state[0][2] + _h*state[1][2] << std::endl;
    //std::cout <<state[0][0]<< " " <<state[0][1] << " " <<state[0][2] << std::endl;
    //std::cout <<state[1][0]<< " " <<state[1][1] << " " <<state[1][2] << std::endl;

    // std::cout <<_h*state[1][0] << " " <<_h*state[1][1] << " " <<_h*state[1][2] << std::endl;

    f.push_back(state[1]);
    f.push_back(g);
    // std::cout << _h*g[0] << " " << _h*g[1] << " "<< _h*g[2] << std::endl;
    // std::cout << f[0][0] << " " << f[0][1] << " " << f[0][2] << std::endl;
    // std::cout << f[1][0] << " " << f[1][1] << " " << f[1][2] << std::endl;

    // std::cout << "_______" << std::endl;

    // std::cout << " -- "<< std::endl;

    // f.push_back(state[1] + 0.5*g*t*t);
    return f;
}

void Particle::updateVelocity(Vector3f vel){
    m_vVecState[1] = vel;
}