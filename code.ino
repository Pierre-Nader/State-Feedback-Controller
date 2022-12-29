// Side note, required libraries -> BasicLinearAlgebra and StateSpace Control

// https://github.com/tomstewart89/BasicLinearAlgebra
// https://github.com/tomstewart89/StateSpaceControl

#include <StateSpaceControl.h>

// objects for 
Model<4, 2, 4> biRotor;
StateSpaceController<4, 2> controller(biRotor);

// sampling frequency of motor, can be changed for our preference
const float dt = 0.01;

// output array for actuators / ESC
float actVals[2];

void setup() {
  
  // initialise model and controller parameters
  
  initModel();
  initController();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void initModel(){
  // <4, 2, 4> in the declaration of the model object is <States, Inputs, Outputs>
  // adding model parameters matrices A, B, C, D
  biRotor.A = {0, 1, 0, 0, 
               0, 0, 0, 0,
               0, 0, 0, 1,
               0, 0, 0, 0};
             
  biRotor.B = {0, 0,  
               13.6992, -13.6992,
               0, 0,
               7.14637, 7.14637};

  biRotor.C = {0, 1, 0, 0, 
               0, 0, 0, 1};

  biRotor.D = 0;
  
}

void initController(){
  // creating controller object
  
  StateSpaceController<4, 2> controller(biRotor);
  
  // <4, 2> in the declaration of the controller object is <States, Inputs>
  // not adding the number of outputs asssumes full state feedback
  
  // adding gains of controller 
  controller.K = {6.16805, 0.948945, 11.8238, 1.81908
                 -6.16805, -0.948945, 11.8238, 1.81908};
                 
  // initialise controller
  controller.initialise();
  
  // setting setpoint for controller
  controller.r = {0, 0, 0, 0};
  
}

void updateAct(float observerValues[], float actVals[]){

  //creating matrix object to take information of observed states from ESP32
  
  Matrix<4> observedStates;
  
  observedStates = {observerValues[0],
                    observerValues[1],
                    observerValues[2],
                    observerValues[3]};

  // updates values of states with the observedStates
  controller.update(observedStates, dt);

  // takes values from controller and are added to an array so that they can go the actuators 
  actVals[0] = controller.u(0);
  actVals[1] = controller.u(1);
  
}
