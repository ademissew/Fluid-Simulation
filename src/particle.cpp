#include "particle.h"
#include "timestepper.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

Particle::Particle(Vector3f vel, Vector3f pos, float h)
{
    // TODO 3.2 initialize the fluid system
    m_vVecState.push_back(pos);
    m_vVecState.push_back(vel);
    t = 0;
    _h = h;
}    

Vector3f g = Vector3f(0,-9.8,0);

std::vector<Vector3f> Particle::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    float t = 0;
    f.push_back(state[1] + g*t);
    f.push_back(state[1] + 0.5*g*t*t);
    t += _h;
    return f;
}

void Particle::updateVelocity(Vector3f vel){
    m_vVecState[1] = vel;
}

// render the system (ie draw the particles)
// void Cell::draw(GLProgram& gl, Vector3f pos)
// {

//     // TODO 3.2: draw the particle. 
//     //           we provide code that draws a static sphere.
//     //           you should replace it with your own
//     //           drawing code.
//     //           In this assignment, you must manage two
//     //           kinds of uniforms before you draw
//     //            1. Update material uniforms (color)
//     //            2. Update transform uniforms
//     //           GLProgram is a helper object that has
//     //           methods to set the uniform state.

//     const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
//     gl.updateMaterial(PARTICLE_COLOR);

//     if (_filled){
//         // Vector3f pos(m_vVecState[0]-Vector3f(5,5,5));
//         gl.updateModelMatrix(Matrix4f::translation(pos));
//         drawSphere(0.075f, 10, 10);
//         // drawSphere(0.5f, 10, 10);
//     }
// }
