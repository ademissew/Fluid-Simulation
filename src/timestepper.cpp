#include "timestepper.h"
#include <iostream>
#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{

    std::vector<Vector3f> x = particleSystem -> getState();
    std::vector<Vector3f> f = particleSystem -> evalF(particleSystem -> getState());
    std::vector<Vector3f> new_state;
    for(int i=0; i<f.size(); ++i){
        new_state.push_back(x[i]+stepSize*f[i]);
    }
    particleSystem -> setState(new_state);
    
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{

    std::vector<Vector3f> x = particleSystem -> getState();
    std::vector<Vector3f> f_0 = particleSystem -> evalF(particleSystem -> getState());
    std::vector<Vector3f> new_state;
    std::vector<Vector3f> input; 
    for(int i=0; i<f_0.size(); ++i){
        input.push_back(x[i]+stepSize*f_0[i]);   
    }
    std::vector<Vector3f> f_1 = particleSystem -> evalF(input);

    for(int i=0; i<f_0.size(); ++i){
        new_state.push_back(x[i]+stepSize*(f_0[i]+f_1[i])/2);     
    }
    particleSystem -> setState(new_state);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{    
                        std::cout << typeid(particleSystem).name() << std::endl;

    std::vector<Vector3f> x = particleSystem -> getState();
                        std::cout << "hi3" << std::endl;

    std::vector<Vector3f> k_1 = particleSystem -> evalF(particleSystem -> getState());
    std::vector<Vector3f> new_state;
    std::vector<Vector3f> input1,input2,input3; 

    for(int i=0; i<x.size(); ++i){
        input1.push_back(x[i]+stepSize*k_1[i]/2);
    } 
    std::vector<Vector3f> k_2 = particleSystem -> evalF(input1);
    for(int i=0; i<x.size(); ++i){
        input2.push_back(x[i]+stepSize*k_2[i]/2);
    }
    std::vector<Vector3f> k_3 = particleSystem -> evalF(input2);
    for(int i=0; i<x.size(); ++i){
        input3.push_back(x[i]+stepSize*k_3[i]);
    }
    std::vector<Vector3f> k_4 = particleSystem -> evalF(input3);
    for(int i=0; i<x.size(); ++i){
        new_state.push_back(x[i]+(stepSize/6)*(k_1[i]+2*k_2[i]+2*k_3[i]+k_4[i]));
    }
    particleSystem -> setState(new_state);

}

