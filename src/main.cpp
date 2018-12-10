#include "gl.h"
#include <GLFW/glfw3.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "vertexrecorder.h"
#include "starter3_util.h"
#include "camera.h"
#include "timestepper.h"
#include "cell.h"
#include "particle.h"
#include <Eigen/Dense>
using Eigen::MatrixXd;

using namespace std;

namespace
{

// Declarations of functions whose implementations occur later.
void initSystem();
void stepSystem();
void drawSystem();
void freeSystem();
void resetTime();

void initRendering();
void drawAxis();

// Some constants
const Vector3f LIGHT_POS(3.0f, 3.0f, 5.0f);
const Vector3f LIGHT_COLOR(120.0f, 120.0f, 120.0f);
const Vector3f FLOOR_COLOR(1.0f, 0.0f, 0.0f);

// time keeping
// current "tick" (e.g. clock number of processor)
uint64_t start_tick;
// number of seconds since start of program
double elapsed_s;
// number of seconds simulated
double simulated_s;

// Globals here.
TimeStepper* timeStepper;
float h;
char integrator;

Camera camera;
bool gMousePressed = false;
GLuint program_color;
GLuint program_light;

std::vector<std::vector<std::vector<Cell>>> grid; //3D grid of cells
vector<Particle> particles;

int n = 10;

// Function implementations
static void keyCallback(GLFWwindow* window, int key,
    int scancode, int action, int mods)
{
    if (action == GLFW_RELEASE) { // only handle PRESS and REPEAT
        return;
    }

    // Special keys (arrows, CTRL, ...) are documented
    // here: http://www.glfw.org/docs/latest/group__keys.html
    switch (key) {
    case GLFW_KEY_ESCAPE: // Escape key
        exit(0);
        break;
    case ' ':
    {
        Matrix4f eye = Matrix4f::identity();
        camera.SetRotation(eye);
        camera.SetCenter(Vector3f(0, 0, 0));
        break;
    }
    case 'R':
    {
        cout << "Resetting simulation\n";
        freeSystem();
        initSystem();
        resetTime();
        break;
    }
    default:
        cout << "Unhandled key press " << key << "." << endl;
    }
}

static void mouseCallback(GLFWwindow* window, int button, int action, int mods)
{
    double xd, yd;
    glfwGetCursorPos(window, &xd, &yd);
    int x = (int)xd;
    int y = (int)yd;

    int lstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    int mstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    if (lstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::LEFT, x, y);
    }
    else if (rstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::RIGHT, x, y);
    }
    else if (mstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::MIDDLE, x, y);
    }
    else {
        gMousePressed = true;
        camera.MouseRelease(x, y);
        gMousePressed = false;
    }
}

static void motionCallback(GLFWwindow* window, double x, double y)
{
    if (!gMousePressed) {
        return;
    }
    camera.MouseDrag((int)x, (int)y);
}

void setViewport(GLFWwindow* window)
{
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    camera.SetDimensions(w, h);
    camera.SetViewport(0, 0, w, h);
    camera.ApplyViewport();
}

void drawAxis()
{
    glUseProgram(program_color);
    Matrix4f M = Matrix4f::translation(camera.GetCenter()).inverse();
    camera.SetUniforms(program_color, M);

    const Vector3f DKRED(1.0f, 0.5f, 0.5f);
    const Vector3f DKGREEN(0.5f, 1.0f, 0.5f);
    const Vector3f DKBLUE(0.5f, 0.5f, 1.0f);
    const Vector3f GREY(0.5f, 0.5f, 0.5f);

    const Vector3f ORGN(0, 0, 0);
    const Vector3f AXISX(5, 0, 0);
    const Vector3f AXISY(0, 5, 0);
    const Vector3f AXISZ(0, 0, 5);

    VertexRecorder recorder;
    recorder.record_poscolor(ORGN, DKRED);
    recorder.record_poscolor(AXISX, DKRED);
    recorder.record_poscolor(ORGN, DKGREEN);
    recorder.record_poscolor(AXISY, DKGREEN);
    recorder.record_poscolor(ORGN, DKBLUE);
    recorder.record_poscolor(AXISZ, DKBLUE);

    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISX, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISY, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISZ, GREY);

    glLineWidth(3);
    recorder.draw(GL_LINES);
}

void solvePressure(){
    MatrixXd d(n*n*n,1);
    MatrixXd m(n*n*n,n*n*n);    
    MatrixXd sol(n*n*n,n*n*n);    


    for(int i=0; i< n; ++i){ // Sets up matrix
        for (int j=0; j < n; ++j){
            for (int k=0; k < n; ++k){
                Vector3f vel = grid[i][j][k]._particle.m_vVecState[1];
                float divergence = -(vel.x()+vel.y()+vel.z())/h ;
                if (i < n-1) {
                    divergence += grid[i+1][j][k]._particle.m_vVecState[1].x()/float(h);
                } if (j < n-1) {
                    divergence += grid[i][j+1][k]._particle.m_vVecState[1].y()/float(h);
                } if (k < n-1) {
                    divergence += grid[i][j][k+1]._particle.m_vVecState[1].z()/float(h);
                } 
                d(i*n*n+j*n+k,0) = divergence;
                int neighbor_particles = 0;
                for(int x=0; x< n; ++x){
                    for (int y=0; y< n; ++y){
                        for (int z=0; z< n; ++z){
                            if (i-1 == x || i+1 == x || j-1 == y || j+1 == y || k-1 == z || k+1 == z){
                                m(i*n*n+j*n+k,x*n*n+y*n+z) = 1;                                
                                if(grid[x][y][z]._filled){
                                    neighbor_particles--;
                                }
                            } else {
                                m(i*n*n+j*n+k,x*n*n+y*n+z) = 0;
                            }
                        }
                    }
                }  
                m(i*n*n+j*n+k,i*n*n+j*n+k) = neighbor_particles;
            }    
        }
    }
    // m = m.inverse();
    // sol = m.inverse()*d;


    // vector<float> pressure(n*n*n,0);
    // vector<float> divergence(n*n*n,0);

    // for (int nx = 0; nx < particles.size(); nx++){
    //     int i = particles[nx].m_vVecState[0][0];
    //     int j = particles[nx].m_vVecState[0][1];
    //     int k = particles[nx].m_vVecState[0][2];
    //     divergence[n*n*i + n*j + k] = particles[nx].m_vVecState[0].y();
    // }

    // for (int i = 1; i < n - 1; i++){
    //     for (int j = 1; j < n - 1; j++){
    //         for (int k = 1; k < n - 1; k++){
    //             divergence[n*n*i + n*j + k] = -0.5*(grid[i+1][j][k]._particle.m_vVecState[1].x() - grid[i-1][j][k]._particle.m_vVecState[1].x() +
    //                 grid[i][j+1][k]._particle.m_vVecState[1].y() - grid[i][j-1][k]._particle.m_vVecState[1].y() +
    //                 grid[i][j][k+1]._particle.m_vVecState[1].z() - grid[i][j][k-1]._particle.m_vVecState[1].z()
    //                 )/n;
    //         }

    //     }

    // }
    // for (int steps = 0; steps < 20; steps++){               
    //     for (int i = 1; i < n-1; ++i){ //adjusted pressure
    //             for (int j = 1; j < n-1; ++j ){
    //                 for (int k = 1; k < n-1; ++k){
    //                     pressure[n*n*i + n*j + k] = 0.25*(divergence[n*n*i + n*j + k] +
    //                     pressure[n*n*(i-1) + n*j + k] + pressure[n*n*(i+1) + n*j + k] +
    //                     pressure[n*n*i +n*(j-1) + k] + pressure[n*n*i + n*(j+1) + k] +
    //                     pressure[n*n*i + n*j + (k-1)] + pressure[n*n*i + n*j + (k+1)]);                    
    //             }
    //         }
    //     }
    // }


    // for (int i = 1; i < n-1; ++i){ //adjusted pressure
    //     for (int j = 1; j < n-1; ++j ){
    //         for (int k = 1; k < n-1; ++k){
    //             Vector3f vel(grid[i][j][k]._particle.m_vVecState[1]);
    //             cout << "vel begin" << endl;
    //                             cout << grid[i][j][k]._particle.m_vVecState[1][0] << " " << vel[1] << " " << vel[2] << endl;

    //             vel -= 0.5*n*Vector3f(pressure[n*n*(i+1) + n*j + k] - pressure[n*n*(i-1) + n*j + k],
    //             pressure[n*n*i + n*(j+1) + k] - pressure[n*n*i + n*(j-1) + k],
    //             pressure[n*n*i + n*j + (k+1)] - pressure[n*n*i + n*j + (k+1)]);
    //             // cout << "vel end" << endl;
    //             // cout << vel[0] << " " << vel[1] << " " << vel[2] << endl;
    //             grid[i][j][k]._particle.updateVelocity(vel); 
    //             // cout << .5*(pressure[n*n*(i+1) + n*j + k] - pressure[n*n*(i-1) + n*j + k]  )/h << endl;
    //             // grid[i][j][k]._particle.m_vVecState[1][0] -= .5*(pressure[n*n*(i+1) + n*j + k] - pressure[n*n*(i-1) + n*j + k]  )/h;
    //             // grid[i][j][k]._particle.m_vVecState[1][1] -= .5*(pressure[n*n*(i) + n*(j+1) + k] - pressure[n*n*i + n*(j-1) + k]  )/h;
    //             // grid[i][j][k]._particle.m_vVecState[1][2] -= .5*(pressure[n*n*(i) + n*(j) + k+1] - pressure[n*n*i + n*(j) + k-1]  )/h;
    //             }
    //     }
    // }
    // std::cout << "____________________" << std::endl;

}


// initialize your particle systems
void initSystem()
{
    switch (integrator) {
    case 'e': timeStepper = new ForwardEuler(); break;
    case 't': timeStepper = new Trapezoidal(); break;
    case 'r': timeStepper = new RK4(); break;
    default: printf("Unrecognized integrator\n"); exit(-1);
    }
    grid.clear();
    // initializes a 11x11 grid
    for (int i = 0; i < n+2; ++i){
        std::vector<std::vector<Cell>> vec;
        for (int j = 0; j < n+2; ++j){
            std::vector<Cell> cells;
            for (int k = 0; k < n+2; ++k){
                Cell cell = Cell(Vector3f(i,j,k),n);

                cells.push_back(cell);
            }
            vec.push_back(cells);
        }
        grid.push_back(vec);
    }
}

void freeSystem() {
    // delete simpleSystem; simpleSystem = nullptr;
    delete timeStepper; timeStepper = nullptr;
    grid.clear();
    particles.clear();
}

void resetTime() {
    elapsed_s = 0;
    simulated_s = 0;
    start_tick = glfwGetTimerValue();
}

// TODO: To add external forces like wind or turbulances,
//       update the external forces before each time step
void stepSystem()
{
    // step until simulated_s has caught up with elapsed_s.
    double holder = simulated_s;
    while (simulated_s < elapsed_s) {
        for (int i = 0; i < particles.size(); ++i){
            timeStepper -> takeStep2(&particles[i],h,n);
        }
        simulated_s += h;

    }
    //  std::cout << particles[i].m_vVecState[0][0]<< " " <<particles[i].m_vVecState[0][1] << " " <<particles[i].m_vVecState[0][2] << std::endl;

    
    for (int i = 0; i < particles.size(); ++i) {
        Vector3f pos = particles[i].m_vVecState[0];
        grid[(int)pos.x()][(int)pos.y()][(int)pos.z()].fill(particles[i]);
    }
    // std::cout << "____________________" << std::endl;

    // cout << "before pressure solve" << endl;
    // cout << particles[0].m_vVecState[1][0] << " " << particles[0].m_vVecState[1][1] << " " << particles[0].m_vVecState[1][2] << endl;
    // cout << particles[1].m_vVecState[1][0] << " " << particles[1].m_vVecState[1][1] << " " << particles[1].m_vVecState[1][2] << endl;

    solvePressure();
    //     cout << "after pressure solve" << endl;

    // cout << particles[0].m_vVecState[1][0] << " " << particles[0].m_vVecState[1][1] << " " << particles[0].m_vVecState[1][2] << endl;
    // cout << particles[1].m_vVecState[1][0] << " " << particles[1].m_vVecState[1][1] << " " << particles[1].m_vVecState[1][2] << endl;

    for (int i = 0; i < particles.size(); ++i) {
        Vector3f pos = particles[i].m_vVecState[0];
        grid[(int)pos.x()][(int)pos.y()][(int)pos.z()].unfill();
    }
}

// Draw the current particle positions
void drawSystem()
{
    // GLProgram wraps up all object that
    // particle systems need for drawing themselves
    GLProgram gl(program_light, program_color, &camera);
    gl.updateLight(LIGHT_POS, LIGHT_COLOR.xyz()); // once per frame

    for (int i = 0; i < particles.size(); ++i){
        Vector3f pos = particles[i].m_vVecState[0];
        // cout << pos[0] << " " << pos[1] << " " << pos[2] << endl;
        Cell* cell = &grid[(int)pos.x()][(int)pos.y()][(int)pos.z()];
        cell->draw(gl);
    }

    // set uniforms for floor
    gl.updateMaterial(FLOOR_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(0, -5.0f, 0));
    // draw floor
    drawQuad(50.0f);
}

//-------------------------------------------------------------------

void initRendering()
{
    // Clear to black
    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}
}

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
// int counter = 0;
int main(int argc, char** argv)
{
    if (argc != 3) {
        printf("Usage: %s <e|t|r> <timestep>\n", argv[0]);
        printf("       e: Integrator: Forward Euler\n");
        printf("       t: Integrator: Trapezoid\n");
        printf("       r: Integrator: RK 4\n");
        printf("\n");
        printf("Try  : %s t 0.001\n", argv[0]);
        printf("       for trapezoid (1ms steps)\n");
        printf("Or   : %s r 0.01\n", argv[0]);
        printf("       for RK4 (10ms steps)\n");
        return -1;
    }

    integrator = argv[1][0];
    h = (float)atof(argv[2]);
    printf("Using Integrator %c with time step %.4f\n", integrator, h);


    GLFWwindow* window = createOpenGLWindow(1024, 1024, "Assignment 3");

    // setup the event handlers
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseCallback);
    glfwSetCursorPosCallback(window, motionCallback);

    initRendering();

    // The program object controls the programmable parts
    // of OpenGL. All OpenGL programs define a vertex shader
    // and a fragment shader.
    program_color = compileProgram(c_vertexshader, c_fragmentshader_color);
    if (!program_color) {
        printf("Cannot compile program\n");
        return -1;
    }
    program_light = compileProgram(c_vertexshader, c_fragmentshader_light);
    if (!program_light) {
        printf("Cannot compile program\n");
        return -1;
    }

    camera.SetDimensions(600, 600);
    camera.SetPerspective(50);
    camera.SetDistance(10);

    // Setup system
    initSystem();
                                    

    // Main Loop
    uint64_t freq = glfwGetTimerFrequency();
    resetTime();
    int number = 0;
    // bool b = true;
    while (!glfwWindowShouldClose(window)) {
    // counter++;
        
        // Clear the rendering window
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (number == 1) {
            int w, h;
            glfwGetWindowSize(window, &w, &h);
            glfwSetWindowSize(window, w - 1, h);
        }
        if (number <= 1) number++;

        setViewport(window);

        if (gMousePressed) {
            drawAxis();
        }

        uint64_t now = glfwGetTimerValue();
        elapsed_s = (double)(now - start_tick) / freq;
        
        // EMITTER
        int range = 2;
        int top = n-2;
        if(particles.size() < 10){
            for (int i = 0; i < range; ++i){
                for (int j = 0; j < range; ++j){
                    particles.push_back(Particle(Vector3f(i,top,j),Vector3f(0,0,0),h,n));
                }
            }
        }

        // Complete a step (includes advecting and satisfying conditions)
        stepSystem();

        // Draw the simulation
        drawSystem();

        // Make back buffer visible
        glfwSwapBuffers(window);

        // Check if any input happened during the last frame
        glfwPollEvents();
    }

    // All OpenGL resource that are created with
    // glGen* or glCreate* must be freed.
    glDeleteProgram(program_color);
    glDeleteProgram(program_light);


    return 0;	// This line is never reached.
}
