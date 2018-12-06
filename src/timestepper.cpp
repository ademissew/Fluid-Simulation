#include "timestepper.h"
#include <iostream>
#include <cstdio>

std::vector<Vector3f> clamp(std::vector<Vector3f> vec, int _n) {
    std::vector<Vector3f> clamp_vec(vec.size());
    int i = 0;
    bool collision = false;
    for (int j = 0; j < 3; ++j){
        if (vec[i][j] > _n-1){
            std::cout << "hi" << std::endl;
            clamp_vec[i][j] = _n-1;
            clamp_vec[i+1][j] = -vec[i+1][j];
            collision = true;
            std::cout << "high clamp" << std::endl;
        } else if (vec[i][j] < 0){
            clamp_vec[i][j] = 0;
            clamp_vec[i+1][j] = -vec[i+1][j];
            collision = true;
                        std::cout << "low clamp" << std::endl;

        } else {
            clamp_vec[i][j] = vec[i][j];
            clamp_vec[i+1][j] = vec[i+1][j];
        }
    }
    if (collision){
        clamp_vec[1] *= 0.1;
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

void RK4::takeStep2(ParticleSystem* particleSystem, float stepSize, int n){
    
    std::vector<Vector3f> state = particleSystem->getState();
    std::vector <Vector3f> k1 = particleSystem->evalF(state);
    std::cout <<"rk4 begins" << std::endl;
     std::cout <<state[0][0]<< " " <<state[0][1] << " " <<state[0][2] << std::endl;
     std::cout <<state[1][0]<< " " <<state[1][1] << " " <<state[1][2] << std::endl;
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
    std::cout <<"rk4 ends" << std::endl;
     std::cout <<newState[0][0]<< " " <<newState[0][1] << " " <<newState[0][2] << std::endl;
     std::cout <<newState[1][0]<< " " <<newState[1][1] << " " <<newState[1][2] << std::endl;
    particleSystem->setState(newState);
}

// void RK4::takeStep2(ParticleSystem* particleSystem, float stepSize, int n)
// {    
//     std::vector<Vector3f> x = particleSystem -> getState();
//     std::cout <<"rk4 begins" << std::endl;
//     std::cout <<x[0][0]<< " " <<x[0][1] << " " <<x[0][2] << std::endl;
//     std::cout <<x[1][0]<< " " <<x[1][1] << " " <<x[1][2] << std::endl;
//     // std::cout << x[1][0] << " " << x[1][1] << " " << x[1][2] << std::endl

//     std::vector<Vector3f> k_1 = particleSystem -> evalF(x);
//     // std::cout << k_1[0][0] << " " << k_1[0][1] << " " << k_1[0][2] << std::endl;

//                             // std::cout << "hi100" << std::endl;

//     std::vector<Vector3f> new_state;
//     std::vector<Vector3f> input1,input2,input3; 

//     for(int i=0; i<x.size(); ++i){
//         input1.push_back(x[i]+stepSize*k_1[i]/2);
//     } 
//     std::vector<Vector3f> k_2 = particleSystem -> evalF(input1);
//     for(int i=0; i<x.size(); ++i){
//         input2.push_back(x[i]+stepSize*k_2[i]/2);
//     }
//                             // std::cout << "hi4" << std::endl;

//     std::vector<Vector3f> k_3 = particleSystem -> evalF(input2);
//     for(int i=0; i<x.size(); ++i){
//         input3.push_back(x[i]+stepSize*k_3[i]);
//     }
//     std::vector<Vector3f> k_4 = particleSystem -> evalF(input3);
//     for(int i=0; i<x.size(); ++i){
//         new_state.push_back(x[i]+(stepSize/6)*(k_1[i]+2*k_2[i]+2*k_3[i]+k_4[i]));
//     }
//     // std::cout << new_state[0][0] << " " << new_state[0][1] << " " << new_state[0][2] << std::endl;
//     // std::cout << new_state[1][0] << " " << new_state[1][1] << " " << new_state[1][2] << std::endl;
//     // std::cout << " " << std::endl;
//     new_state = clamp(new_state,n);
//     // std::cout << new_state[0][0] << " " << new_state[0][1] << " " << new_state[0][2] << std::endl;
//     std::cout << "___________" << std::endl;

//     particleSystem -> setState(new_state);
//         std::cout <<"rk4 ends" << std::endl;

//     std::cout <<new_state[0][0]<< " " <<new_state[0][1] << " " <<new_state[0][2] << std::endl;
//     std::cout <<new_state[1][0]<< " " <<new_state[1][1] << " " <<new_state[1][2] << std::endl;
//     // std::cout << particleSystem->getState()[1][0] << " " << particleSystem->getState()[1][1] << " " << particleSystem->getState()[1][2] <<std::endl;

//     // // std::cout << new_state[0][0] << " " << new_state[0][1] << " " << new_state[0][2] << std::endl;
//     // // std::cout << new_state[1][0] << " " << new_state[1][1] << " " << new_state[1][2] << std::endl;
//     // std::cout << particleSystem -> getState()[1][0] << " " << particleSystem -> getState()[1][1] << " " << particleSystem -> getState()[1][2] << std::endl;



//     // std::cout << "hi5" << std::endl;

// }
