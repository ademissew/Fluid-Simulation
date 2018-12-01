#include "cell.h"
#include "timestepper.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

Cell::Cell(Vector3f vel, Vector3f pos, float h)
{
    // TODO 3.2 initialize the fluid system
    m_vVecState.push_back(pos);
    m_vVecState.push_back(vel);
    next_state.push_back(pos);
    next_state.push_back(vel);
    float _h = h;
}    

Vector3f g = Vector3f(0,-9.8,0);

std::vector<Vector3f> Cell::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    f.push_back((state[0] + state[1]*_h + 0.5*g*_h*_h)); // if not filled the new velocity is just 0

    f.push_back(state[1] + g*_h);
    return f;
}

void Cell::fill(){
    _filled = true;
}

void Cell::unfill(){
    _filled = false;
}

void Cell::setState(std::vector<Vector3f> &state){
    next_state = state;
}

// render the system (ie draw the particles)
void Cell::draw(GLProgram& gl)
{

    // TODO 3.2: draw the particle. 
    //           we provide code that draws a static sphere.
    //           you should replace it with your own
    //           drawing code.
    //           In this assignment, you must manage two
    //           kinds of uniforms before you draw
    //            1. Update material uniforms (color)
    //            2. Update transform uniforms
    //           GLProgram is a helper object that has
    //           methods to set the uniform state.

    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);

    if (true || _filled){
        Vector3f pos(m_vVecState[0]);
        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(0.075f, 10, 10);
    }
}
