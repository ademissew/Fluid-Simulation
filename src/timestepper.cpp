#include "timestepper.h"
#include <iostream>
#include <cstdio>

std::vector<Vector3f> clamp(std::vector<Vector3f> vec, int _n, std::vector<std::vector<std::vector<float>>> grid_x, 
std::vector<std::vector<std::vector<float>>> grid_y,std::vector<std::vector<std::vector<float>>> grid_z) {
    std::vector<Vector3f> clamp_vec(vec.size());
    int i = 0;
    bool collision = false;
    int x = vec[0].x();
    int y = vec[0].y();
    int z = vec[0].z();
    if (vec[0][0] > _n-1){
            clamp_vec[0][0] = _n-1;
            grid_x[_n][y][z] = -grid_x[_n][y][z] *.5;
            collision = true;
    } else if (vec[0][0] < 0){
        clamp_vec[0][0] = 0;
        grid_x[0][y][z] = -grid_x[0][y][z] *.5;
        collision = true;
    } else {
        clamp_vec[0][0] = vec[0][0];
    }
    
    if (vec[0][1] > _n-1){
            clamp_vec[0][1] = _n-1;
            grid_y[x][_n][z] = -grid_y[x][_n][z] *.5;
            collision = true;
    } else if (vec[0][1] < 0){
        clamp_vec[0][1] = 0;
        grid_y[x][0][z] = -grid_y[x][0][z] *.5;
        collision = true;
    } else {
        clamp_vec[0][1] = vec[0][1];
    }

    if (vec[0][2] > _n-1){
            clamp_vec[0][2] = _n-1;
            grid_z[x][y][_n] = -grid_z[x][y][_n] *.5;
            collision = true;
    } else if (vec[0][2] < 0){
        clamp_vec[0][2] = 0;
        grid_z[x][y][z] = -grid_z[x][y][0] *.5;
        collision = true;
    } else {
        clamp_vec[0][2] = vec[0][2];
    }


    return clamp_vec;
    
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{    

}

void RK4::takeStep2(ParticleSystem* particleSystem, float stepSize, int n, std::vector<std::vector<std::vector<float>>> grid_x, 
std::vector<std::vector<std::vector<float>>> grid_y,std::vector<std::vector<std::vector<float>>> grid_z){
    
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
    // newState = clamp(newState,n, grid_x, grid_y, grid_z);
    // std::cout <<"rk4 ends" << std::endl;
     std::cout <<newState[0][0]<< " " <<newState[0][1] << " " <<newState[0][2] << std::endl;
    //  std::cout <<newState[1][0]<< " " <<newState[1][1] << " " <<newState[1][2] << std::endl;
    particleSystem->setState(newState);

}