#include "cell.h"
#include "timestepper.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

Cell::Cell(Vector3f pos, int n)
{
    _pos = pos;
    _n = n;
}    

void Cell::fill(Particle  particle){
    _particle = particle;
    std::cout << "____________________" << std::endl;
    std::cout << _particle.m_vVecState[0][0] << " " << _particle.m_vVecState[0][1] << " " << _particle.m_vVecState[0][2] << std::endl;
    std::cout << "____________________" << std::endl;

}

void Cell::unfill(){
    _particle = Particle(Vector3f(0,0,0),Vector3f(0,0,0),0,1); //empty particle

}

// render the system (ie draw the cell)
void Cell::draw(GLProgram& gl)
{

    // TODO 3.2: draw the cell. 
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
    gl.updateModelMatrix(Matrix4f::translation(_pos/_n));
    drawSphere(0.075f, 10, 10);
}
