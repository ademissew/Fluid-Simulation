#include "pendulumsystem.h"
#include "timestepper.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
std::vector<float> rest_lengths; // holds all springs connected to particle at index as Vector4f(int p2,int restlen,float k,bool fixed), where p2 is index into m_vVecState
PendulumSystem::PendulumSystem()
{
    // TODO 4.2 Add particles for simple pendulum
    for(int i = 0; i < NUM_PARTICLES; ++i){
            m_vVecState.push_back(Vector3f(rand_uniform(-1.0, 0),rand_uniform(0.5, 1.5),rand_uniform(-0.5, 0.5))); // x
            m_vVecState.push_back(Vector3f(0,0,0)); // v
            if (i>0){
                Vector3f dist = m_vVecState[i] - m_vVecState[i-2];
                rest_lengths.push_back(dist.abs()/5);
        }
    }
    
    // TODO 4.3 Extend to multiple particles
    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    // TODO 4.1: implement evalF
    //  - gravity
    //  - viscous drag
    //  - springs

    std::vector<Vector3f> f(state.size());
    float m = 1.0;
    Vector3f g = Vector3f(0,-9.8,0);
    float k_drag = 0.8;
    float k_spring = 25;
    
    for(int i=0;i<state.size()-1;i+=2){ //starts with 0,1
        f[i] = state[i+1];
        if (i==0){
            f[i+1] = Vector3f(0,0,0);
        } else {
            Vector3f force = m*g - k_drag*state[i+1];
            Vector3f d_1 = state[i]-state[i-2];
            force -= k_spring*(d_1.abs() - rest_lengths[i/2-1])*(d_1/d_1.abs());
             if (i != state.size()-2){
                Vector3f d = state[i]-state[i+2];
                force -= k_spring*(d.abs() - rest_lengths[i/2])*(d/d.abs());
            }
            f[i+1] = force;
        }
    }
    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
    gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
    // drawSphere(0.075f, 10, 10);
    for (int i=0; i< m_vVecState.size(); i+=2){
        Vector3f pos(m_vVecState[i]);
        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(0.075f, 10, 10);
    }
    
}
