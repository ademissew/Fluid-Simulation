#include "timestepper.h"
#include <iostream>
#include <cstdio>

std::vector<Vector3f> clamp(std::vector<Vector3f> vec, int _n) {
    std::vector<Vector3f> clamp(vec.size());
    for (int i = 0; i<vec.size(); ++i){
        for (int j = 0; j < 3; ++j){
            // if (i == 0){ //position
            if (vec[i][j] > _n-1){
                clamp[i][j] = _n - 1;
            } if (vec[i][j] < 0){
                clamp[i][j] = 0;
            } else {
                clamp[i][j] = vec[i][j];
            }
            // }
            //  else { //velocity
            //     if (vec[i][j] > _n-1){
            //         clamp[i][j] = _n - 1;
            //     } if (vec[i][j] < 0){
            //         clamp[i][j] = 0;
            //     } else {
            //         clamp[i][j] = vec[i][j];
            //     }

            // }
            // if (vec[i][j] < 0 && i == 0){
            //     clamp[i][j] = 0;
            // } else if (vec[i][j] > _n - 1 && i == 0) {
            //     clamp[i][j] = _n-1;            
            // } else {
            //     clamp[i][j] = vec[i][j];
            // }
        }
    }
    return clamp;
    
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
                        // std::cout << typeid(particleSystem).name() << std::endl;

    std::vector<Vector3f> x = particleSystem -> getState();
                        // std::cout << x.size() << std::endl;
                        // std::cout << particleSystem -> m_vVecState.size() << std::endl;


    std::vector<Vector3f> k_1 = particleSystem -> evalF(x);
                            // std::cout << "hi100" << std::endl;

    std::vector<Vector3f> new_state;
    std::vector<Vector3f> input1,input2,input3; 

    for(int i=0; i<x.size(); ++i){
        input1.push_back(x[i]+stepSize*k_1[i]/2);
    } 
    std::vector<Vector3f> k_2 = particleSystem -> evalF(input1);
    for(int i=0; i<x.size(); ++i){
        input2.push_back(x[i]+stepSize*k_2[i]/2);
    }
                            // std::cout << "hi4" << std::endl;

    std::vector<Vector3f> k_3 = particleSystem -> evalF(input2);
    for(int i=0; i<x.size(); ++i){
        input3.push_back(x[i]+stepSize*k_3[i]);
    }
    std::vector<Vector3f> k_4 = particleSystem -> evalF(input3);
    for(int i=0; i<x.size(); ++i){
        new_state.push_back(x[i]+(stepSize/6)*(k_1[i]+2*k_2[i]+2*k_3[i]+k_4[i]));
    }
    // new_state = clamp(new_state,n);
    particleSystem -> setState(new_state);

    // std::cout << new_state[0][0] << " " << new_state[0][1] << " " << new_state[0][2] << std::endl;
    // std::cout << new_state[1][0] << " " << new_state[1][1] << " " << new_state[1][2] << std::endl;

    // std::cout << "hi5" << std::endl;

}
void RK4::takeStep2(ParticleSystem* particleSystem, float stepSize, int n)
{    
    std::vector<Vector3f> x = particleSystem -> getState();
    // std::cout << x[1][0] << " " << x[1][1] << " " << x[1][2] << std::endl

    std::vector<Vector3f> k_1 = particleSystem -> evalF(x);
                            // std::cout << "hi100" << std::endl;

    std::vector<Vector3f> new_state;
    std::vector<Vector3f> input1,input2,input3; 

    for(int i=0; i<x.size(); ++i){
        input1.push_back(x[i]+stepSize*k_1[i]/2);
    } 
    std::vector<Vector3f> k_2 = particleSystem -> evalF(input1);
    for(int i=0; i<x.size(); ++i){
        input2.push_back(x[i]+stepSize*k_2[i]/2);
    }
                            // std::cout << "hi4" << std::endl;

    std::vector<Vector3f> k_3 = particleSystem -> evalF(input2);
    for(int i=0; i<x.size(); ++i){
        input3.push_back(x[i]+stepSize*k_3[i]);
    }
    std::vector<Vector3f> k_4 = particleSystem -> evalF(input3);
    for(int i=0; i<x.size(); ++i){
        new_state.push_back(x[i]+(stepSize/6)*(k_1[i]+2*k_2[i]+2*k_3[i]+k_4[i]));
    }
    // std::cout << new_state[0][0] << " " << new_state[0][1] << " " << new_state[0][2] << std::endl;
    // std::cout << new_state[1][0] << " " << new_state[1][1] << " " << new_state[1][2] << std::endl;
    // std::cout << " " << std::endl;
    new_state = clamp(new_state,n);
    // std::cout << new_state[1][0] << " " << new_state[1][1] << " " << new_state[1][2] << std::endl;

    particleSystem -> setState(new_state);
    // std::cout << particleSystem->getState()[1][0] << " " << particleSystem->getState()[1][1] << " " << particleSystem->getState()[1][2] <<std::endl;

    // // std::cout << new_state[0][0] << " " << new_state[0][1] << " " << new_state[0][2] << std::endl;
    // // std::cout << new_state[1][0] << " " << new_state[1][1] << " " << new_state[1][2] << std::endl;
    // std::cout << particleSystem -> getState()[1][0] << " " << particleSystem -> getState()[1][1] << " " << particleSystem -> getState()[1][2] << std::endl;



    // std::cout << "hi5" << std::endl;

}
