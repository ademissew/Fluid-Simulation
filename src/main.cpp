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
#include "clothsystem.h"


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
int size = 10;
float delta_t;
float diff;
float visc;

vector<float> s(size*size*size);
vector<float> density(size*size*size);
vector<float> x_vel(size*size*size);
vector<float> y_vel(size*size*size);
vector<float> z_vel(size*size*size);
vector<float> x_vel0(size*size*size);
vector<float> y_vel0(size*size*size);
vector<float> z_vel0(size*size*size);

Camera camera;
bool gMousePressed = false;
GLuint program_color;
GLuint program_light;

ClothSystem* clothSystem;


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

int value(int i, int j, int k){
    if (i < 0 || j < 0 || k < 0 || i > size-1 || j > size - 1 || k > size -1){
        return 0;
    } return k + size*j + size*size*i;
}

void addFluid(int i, int j, int k, float fluidAmount){
    density[value(i,j,k)] = fluidAmount;
}

void adjustVelocity(int i, int j, int k, float velx, float vely, float velz){
    x_vel[value(i,j,k)] += velx;
    y_vel[value(i,j,k)] += vely;
    z_vel[value(i,j,k)] += velz;
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


// initialize your particle systems
void initSystem()
{
    switch (integrator) {
    case 'e': timeStepper = new ForwardEuler(); break;
    case 't': timeStepper = new Trapezoidal(); break;
    case 'r': timeStepper = new RK4(); break;
    default: printf("Unrecognized integrator\n"); exit(-1);
    }
    int size = 10;
    float delta_t = h;
    float diff = 5.0;
    float visc = 5.0;

    s = vector<float>(size*size*size,0);
    density = vector<float>(size*size*size,0);
    x_vel = vector<float>(size*size*size,0);
    y_vel = vector<float>(size*size*size,0);
    z_vel = vector<float>(size*size*size,0);
    x_vel0 = vector<float>(size*size*size,0);
    y_vel0 = vector<float>(size*size*size,0);
    z_vel0 = vector<float>(size*size*size,0);

}

void freeSystem() {
    // delete simpleSystem; simpleSystem = nullptr;
    s.clear();
    density.clear();
    x_vel.clear();
    y_vel.clear();
    z_vel.clear();
    x_vel0.clear();
    y_vel0.clear();
    z_vel0.clear();
}

void resetTime() {
    elapsed_s = 0;
    simulated_s = 0;
    start_tick = glfwGetTimerValue();
}

// TODO: To add external forces like wind or turbulances,
//       update the external forces before each time step
void set_bnd(int condition, vector<float> input){
    for (int b = 1; b<size-1; b++){
        for(int a = 1; a<size-1; a++){
            if (condition == 3){ // means flip when hitting x boundaries
                input[value(b,a,0)] = -input[value(b,a,1)];
                input[value(b,a,size-1)] = -input[value(b,a,size-2)];
            } else {
                input[value(b,a,0)] = input[value(b,a,1)];
                input[value(b,a,size-1)] = input[value(b,a,size-2)];
            } if (condition == 2){// means flip when hitting y boundaries
                input[value(b,0,a)] = -input[value(b,1,a)];
                input[value(b,size-1,a)] = -input[value(b,size-2,a)];
            } else{
                input[value(b,0,a)] = input[value(b,1,a)];
                input[value(b,size-1,a)] = input[value(b,size-2,a)];
            } if (condition == 1){ // means flip when hitting z boundaries
                input[value(0,b,a)] = -input[value(1,b,a)];
                input[value(size-1,b,a)] = -input[value(size-2,b,a)];
            } else {
                input[value(0,b,a)] = input[value(1,b,a)];
                input[value(size-1,b,a)] = input[value(size-2,b,a)];
            }
        }
    }
    // avg at corner
    input[value(0,0,0)] = (input[value(1,0,0)]+input[value(0,1,0)]+input[value(0,1,0)])/3.0f;
    input[value(0,0,size-1)] = (input[value(1,0,size-1)]+input[value(0,1,size-1)]+input[value(0,0,size)])/3.0f;
    input[value(0,size-1,0)] = (input[value(1,size-1,0)]+input[value(0,size-1,0)]+input[value(0,size-1,1)])/3.0f;
    input[value(0,size-1,size-1)] = (input[value(1,size-1,size-1)]+input[value(0,size-2,size-1)]+input[value(0,size-1,size-2)])/3.0f;
    input[value(size-1,0,0)] = (input[value(size-2,0,0)]+input[value(size-1,1,0)]+input[value(size-1,0,1)])/3.0f;
    input[value(size-1,0,size-1)] = (input[value(size-2,0,size-1)]+input[value(size-1,1,size-1)]+input[value(size-1,0,size-2)])/3.0f;
    input[value(size-1,size-1,0)] = (input[value(size-2,size-1,0)]+input[value(size-1,size-2,1)]+input[value(size-1,size-1,1)])/3.0f;
    input[value(size-1,size-1,size-1)] = (input[value(size-2,size-1,size-1)]+input[value(size-1,size-2,size-1)]+input[value(size-1,size-1,size-2)])/3.0f;

}

void solveSystem(int condition, vector<float> input, vector<float> initial_input,float p, float q, int rounds){
    float c_inv = 1.0/q;
    for (int round = 0; round < rounds; ++round){
        for(int i = 1; i < size-1; ++i){
            for(int j = 1; j< size-1; ++j){
                for(int k = 1; k<size-1; ++k){
                    input[value(i,j,k)] = (
                        initial_input[value(i,j,k)] +
                        p * c_inv * (
                        input[value(i-1,j,k)] + input[value(i+1,j,k)] +
                        input[value(i,j-1,k)] + input[value(i,j+1,k)] +
                        input[value(i,j,k-1)] + input[value(i,j,k+1)])
                    );
                }
            }
        }
        set_bnd(condition, input);

    }

}

void diffuseSystem(int condition, vector<float> input, vector<float> initial_input,int rounds){
    float diffusion_value = delta_t*diff*(size-2)*(size-2);
    solveSystem(condition,input, initial_input, diffusion_value,1+6*diffusion_value,rounds);
}

void inforceIncompressibility(vector<float> velx,vector<float> vely,vector<float> velz,vector<float> velx0,vector<float> vely0,int rounds){
    for(int i = 1; i < size-1; ++i){
        for(int j = 1; j< size-1; ++j){
            for(int k = 1; k<size-1; ++k){
                vely0[value(i,j,k)] = -0.5*(
                    velx[value(i+1,j,k)] - velx[value(i-1,j,k)]+
                    vely[value(i,j+1,k)] - vely[value(i,j-1,k)]+
                    velz[value(i,j,k+1)] - velz[value(i,j,k-1)]) / size;
                velx0[value(i,j,k)] = 0;
            }
        }
    }
    set_bnd(0,vely0);
    set_bnd(0,velx0);
    solveSystem(0,velx0,vely0,1,6,rounds);
    for(int i = 1; i < size-1; ++i){
        for(int j = 1; j< size-1; ++j){
            for(int k = 1; k<size-1; ++k){
                velx[value(i,j,k)] = -0.5 * size * (velx0[value(i+1,j,k)]-velx0[value(i-1,j,k)]);
                vely[value(i,j,k)] = -0.5 * size * (velx0[value(i,j+1,k)]-velx0[value(i,j-1,k)]);
                velz[value(i,j,k)] = -0.5 * size * (velx0[value(i,j,k+1)]-velx0[value(i-1,j,k-1)]);
            }
        }
    }
    set_bnd(1,velx);
    set_bnd(2,vely);
    set_bnd(3,velz);
}

void advectFluid(int condition, vector<float> input, vector<float> initial_input,  vector<float> velx, vector<float> vely, vector<float> velz){
    int i, j, k, i0, j0, k0, i1, j1, k1;
    float x, y, z, s0, t0, s1, t1, u0, u1, dt0;
    dt0 = delta_t*size;
    for ( i=1 ; i<=size ; i++ ) {
        for ( j=1 ; j<=size ; j++ ) {
            for ( k=1 ; k<=size ; k++ ) {
                x = i-dt0*velx[value(i,j,k)]; 
                y = j-dt0*vely[value(i,j,k)];
                z = k-dt0*velz[value(i,j,k)];
                if (x<0.5) x=0.5; if (x>size+0.5) x=size+ 0.5; i0=(int)x; i1=i0+1;
                if (y<0.5) y=0.5; if (y>size+0.5) y=size+ 0.5; j0=(int)y; j1=j0+1;
                if (k<0.5) z=0.5; if (k>size+0.5) z=size+ 0.5; k0=(int)z; k1=k0+1;

                s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1; u1 = z-k0; u0 = 1-u1;

                input[value(i,j,k)] = s0 * ( t0 * (u0 * initial_input[value(i0, j0, k0)]
                                +u1 * initial_input[value(i0, j0, k1)])
                                +( t1 * (u0 * initial_input[value(i0, j1, k0)]
                                +u1 * initial_input[value(i0, j1, k1)])))
                                +s1 * ( t0 * (u0 * initial_input[value(i1, j0, k0)]
                                +u1 * initial_input[value(i1, j0, k1)])
                                +( t1 * (u0 * initial_input[value(i1, j1, k0)]
                                +u1 * initial_input[value(i1, j1, k1)])));
            }
        }
    }
    set_bnd (condition, input);
}

void stepSystem()
{
    diffuseSystem(1, x_vel0, x_vel, 4);
    diffuseSystem(2, y_vel0, y_vel, 4);
    diffuseSystem(3, z_vel0, z_vel, 4);

    inforceIncompressibility(x_vel0, y_vel0, z_vel0, x_vel, y_vel, 4);

    advectFluid(1, x_vel, x_vel0, x_vel0, y_vel0, z_vel0);
    advectFluid(2, y_vel, y_vel0, x_vel0, y_vel0, z_vel0);
    advectFluid(3, z_vel, z_vel0, x_vel0, y_vel0, z_vel0);

    inforceIncompressibility(x_vel, y_vel, z_vel, x_vel0, y_vel0, 4);
    
    diffuseSystem(0, s, density, 4);

    advectFluid(0, density, s, x_vel, y_vel, y_vel);

}




// Draw the current particle positions
void drawSystem()
{
    // GLProgram wraps up all object that
    // particle systems need for drawing themselves
    GLProgram gl(program_light, program_color, &camera);
    gl.updateLight(LIGHT_POS, LIGHT_COLOR.xyz()); // once per frame
    int k = 0;
    for (int i = 0; i<size; ++i){
        for (int j = 0; j<size; ++j){
            float d = density[value(i,j,k)];
            if(d>0){
                const Vector3f PARTICLE_COLOR = Vector3f(0.4f, 0.7f, 1.0f)*density[value(i,j,k)];
                gl.updateMaterial(PARTICLE_COLOR);
                gl.updateModelMatrix(Matrix4f::translation(Vector3f(i,j,k)/size));
                drawSphere(0.045f, 10, 10);
            } else {
                const Vector3f PARTICLE_COLOR = Vector3f(0.4f, 0.7f, 1.0f)*0.1;
                gl.updateMaterial(PARTICLE_COLOR);
                gl.updateModelMatrix(Matrix4f::translation(Vector3f(i,j,k)/size));
                drawSphere(0.045f, 10, 10);
            }
            
        }
    }

    // simpleSystem->draw(gl);

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

    // Setup particle system
    initSystem();

    // Main Loop
    uint64_t freq = glfwGetTimerFrequency();
    resetTime();
    int number = 0;
    int size = 10;
    for (int i = 0; i < 2; ++i){
        for(int j = 0; j < 2; ++j){
            for (int k = 0; k <2; ++k){
                addFluid(i,j,k,(rand()%10));
                adjustVelocity(i,j,k,2,-9.8,4);
            }
        }
    }
    while (!glfwWindowShouldClose(window)) {
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
