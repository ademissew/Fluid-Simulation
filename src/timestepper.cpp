#include "timestepper.h"
#include <iostream>
#include <cstdio>

std::vector<Vector3f> clamp(std::vector<Vector3f> vec, int _n) {
    std::vector<Vector3f> clamp_vec(vec.size());
    int i = 0;
    bool collision = false;
    for (int j = 0; j < 3; ++j){
        if (vec[i][j] > _n-2){
            clamp_vec[i][j] = _n-2;
            clamp_vec[i+1][j] = -vec[i+1][j];
            collision = true;
        } else if (vec[i][j] < 1){
            clamp_vec[i][j] = 1;
            clamp_vec[i+1][j] = -vec[i+1][j];
            collision = true;

        } else {
            clamp_vec[i][j] = vec[i][j];
            clamp_vec[i+1][j] = vec[i+1][j];
        }
    }
    if (collision){
        clamp_vec[1] *= .5;
    }
    return clamp_vec;
    
}

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
void ForwardEuler::takeStep2(ParticleSystem* particleSystem, float stepSize, int n){
    std::vector<Vector3f> x = particleSystem -> getState();
    std::vector<Vector3f> f = particleSystem -> evalF(particleSystem -> getState());
    std::vector<Vector3f> new_state;
    for(int i=0; i<f.size(); ++i){
        new_state.push_back(x[i]+stepSize*f[i]);
    }
    //new_state = clamp(new_state,n);
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
void Trapezoidal::takeStep2(ParticleSystem* particleSystem, float stepSize, int n){
}



void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{    

}

void RK4::takeStep2(ParticleSystem* particleSystem, float stepSize, int n){
    
    std::vector<Vector3f> state = particleSystem->getState();
    std::vector <Vector3f> k1 = particleSystem->evalF(state);
    // std::cout <<"rk4 begins" << std::endl;
    //  std::cout <<state[0][0]<< " " <<state[0][1] << " " <<state[0][2] << std::endl;
    //  std::cout <<state[1][0]<< " " <<state[1][1] << " " <<state[1][2] << std::endl;
    std::vector<Vector3f> carry;
    for (int i = 0; i < state.size(); i++){
        carry.push_back(state[i]+(stepSize*k1[i])/2);
    }
    std::vector<Vector3f> k2 = particleSystem -> evalF(carry);

    std::vector<Vector3f> carry2;
    for (int i = 0; i < state.size(); i++){
        carry2.push_back(state[i] + stepSize*(k2[i])/2);
    }
    std::vector<Vector3f> k3 = particleSystem -> evalF(carry2);
    
    std::vector<Vector3f> carry3;
    for (int i = 0; i < state.size(); i++){
        carry3.push_back(state[i] + stepSize*k3[i]);
    }
    std::vector<Vector3f> k4 = particleSystem -> evalF(carry3);
    
    std::vector<Vector3f> newState;
    for (int i = 0; i < state.size(); i++){
        newState.push_back(state[i] + stepSize*(k1[i]+k2[i]+k3[i]+k4[i])/6);
    }
    newState = clamp(newState,n);
    // std::cout <<"rk4 ends" << std::endl;
    //  std::cout <<newState[0][0]<< " " <<newState[0][1] << " " <<newState[0][2] << std::endl;
    //  std::cout <<newState[1][0]<< " " <<newState[1][1] << " " <<newState[1][2] << std::endl;
    particleSystem->setState(newState);
}