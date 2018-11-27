#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"


 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;

std::vector<std::vector<int>> structural; 
std::vector<std::vector<int>> shear; 
std::vector<std::vector<int>> flexion; 


ClothSystem::ClothSystem()
{
    structural.clear();
    shear.clear();
    flexion.clear();
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
    float y = -0.2;
    int counter = 0;
    for (int i= 0; i<W; ++i){
        y += 0.2;
        float x = 0;
        for (int j=0; j<H; ++j){
            m_vVecState.push_back(Vector3f(x,y,0));
            m_vVecState.push_back(Vector3f(0,0,0));
            std::vector<int> neighbors1,neighbors2,neighbors3;
            if (x > 0) {
                neighbors1.push_back(counter-1);
            } if (y < (H-1)*.2) {
                neighbors1.push_back(counter+H);
            } if (x < (W-1)*.2){
                neighbors1.push_back(counter+1);
            } if (y > 0) {
                neighbors1.push_back(counter-H);
            } if (x > 0 && y > 0) {
                neighbors2.push_back(counter-1-W);
            } if (x > 0 && y < (H-1)*.2) {
                neighbors2.push_back(counter-1+H);
            } if (x < (W-1)*.2 && y < (H-1)*.2) {
                neighbors2.push_back(counter+1+H);
            } if (x < (W-1)*.2 && y > 0) {
                neighbors2.push_back(counter+1-H);
            } if (x+2 < (W-1)*.2){
                neighbors3.push_back(counter+2);
            } if (x-2 > 0){
                neighbors3.push_back(counter-2);                
            } if (y+2 < (H-1)*.2){
                neighbors3.push_back(counter+2*H);                
            } if (y-2 > 0){
                neighbors3.push_back(counter-2*H);                                
            }

            x += 0.2;
            counter++;
            structural.push_back(neighbors1);
            shear.push_back(neighbors2);
            flexion.push_back(neighbors3);
        }
    }

}


std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f(state.size());
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
    float m = 1.0;
    Vector3f g = Vector3f(0,-9.8,0);
    float k_drag = 1;
    float k_spring = 350;
    float struct_spring_len = .2;
    float shear_spring_len = .2*powf(2,.5);
    float flex_spring_len = .4;

    
    for(int i=0;i<state.size()-1;i+=2){ 
        f[i] = state[i+1];
        if (i/2==W*H-1 || i/2==W*H-W){ // top two corners
            f[i+1] = Vector3f(0,0,0);
        } else {
            Vector3f force = m*g - k_drag*state[i+1];
            for (int index1: structural[i/2]){
                Vector3f d_1 = state[i]-state[index1*2];
                force -= k_spring*(d_1.abs() - struct_spring_len)*(d_1/d_1.abs());
            } 
            for (int index2: shear[i/2]){
                Vector3f d_2 = state[i]-state[index2*2];
                force -= k_spring*(d_2.abs() - shear_spring_len)*(d_2/d_2.abs());
            } 
            for (int index3: flexion[i/2]){
                Vector3f d_3 = state[i]-state[index3*2];
                force -= k_spring*(d_3.abs() - flex_spring_len)*(d_3/d_3.abs());
            }
            f[i+1] = force;
        }
    }
    return f;
}


void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    float w = 0.2f;
    
    for (int i=0; i<m_vVecState.size(); i+=2){
        gl.updateModelMatrix(Matrix4f::translation(m_vVecState[i]));
        drawSphere(0.04f, 8, 8);
        VertexRecorder rec;
        gl.disableLighting();
        gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
        for(int index: structural[i/2]){
            rec.record(m_vVecState[index*2], CLOTH_COLOR); //below
            rec.record(m_vVecState[i], CLOTH_COLOR);
        }
        glLineWidth(3.0f);
        rec.draw(GL_LINES);
        gl.enableLighting(); // reset to default lighting model

    }

    
    // EXAMPLE END
}