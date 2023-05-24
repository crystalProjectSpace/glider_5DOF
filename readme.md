#What is it
Small console utility for predicting fligt dynamics of unpowered glider
5 dof (3 coordinates + pitch + roll)

##Example of initial data:
node index state=225,1,0,0,0,15,14500,0,0,0 pitch=0 roll=0 tMax=10 dT=0.001 resSparse=50
Where:
1. state - vector of initial trajectory params
2. pitch - initial deflection of elevator
3. roll - initial deflection of ailerons
4. tMax - time of modelling interval
5. dT - integration step
6. resSparse - inerval between two points in result output

##state vector layout
state layout - V, Th, Psi, epsZ, epsX, X, Y, Z, alpha, gamma
