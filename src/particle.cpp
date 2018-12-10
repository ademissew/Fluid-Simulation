#include "particle.h"
#include "timestepper.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>
using namespace std;

Particle::Particle(Vector3f pos, Vector3f vel, float h, int n, std::vector<std::vector<std::vector<float>>> grid_x, 
std::vector<std::vector<std::vector<float>>> grid_y,std::vector<std::vector<std::vector<float>>> grid_z)
{
    // TODO 3.2 initialize the fluid system
    m_vVecState.push_back(pos);
    _h = h;
    x_vel = grid_x;
    y_vel = grid_y;
    z_vel = grid_z;
    int x = pos.x();
    int y = pos.y();
    int z = pos.z();
    m_vVecState.push_back(Vector3f(x_vel[x][y][z],y_vel[x][y][z],z_vel[x][y][z]));
}   

Vector3f g = Vector3f(0,-9.8,0);

std::vector<Vector3f> Particle::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    int x = max(0.0f,state[0].x());
    int y = max(0.0f,state[0].y());
    int z = max(0.0f,state[0].z());
    f.push_back(Vector3f(x_vel[x][y][z],y_vel[x][y][z],z_vel[x][y][z]));
    f.push_back(g);
    return f;
}

void Particle::updateVelocity(Vector3f vel){
    m_vVecState[1] = vel;
}