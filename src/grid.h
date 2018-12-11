#ifndef GRID_H
#define GRID_H

#include <vector>

class Grid
{
    int size;
    float delta_t;
    // float diff;
    // float visc;
    
    float *s;
    float *pressure;
    
    float *x_vel; 
    float *y_vel; 
    float *z_vel;

    float *x_vel0;
    float *y_vel0;
    float *z_vel0;

    void init(int size,float delta_t);
    void freeSystem();


};


#endif
