//#ifndef _NR_h
//#define _NR_h
//#define pi 3.14
//class NR{
//  public:
//
// double BP1[3] = {40, 0, 20};
// double BP2[3] = {340,0,0};
// double BP3[3] = {0,0,300};
//
// double MP1[3] = {0, 0, 0};
// double MP2[3] = {60,0,0};
// double MP3[3] = {0,0,60};
//
//  
//  float f(float x){
//  t[0]=rho[0]*rho[0] - ((x[0] - BP1[0])^2 + (x[1] - BP1[2])^2);
//  t[1]=rho[1]*rho[1] - ((x[1] + MP2[0]*cos(pi/180*x[2]) - BP2[0])^2 + (x[1] + MP2[2]*sin(pi/180*x[2]) - BP2[2])^2)s;
//  t[2]=rho[2]*rho[2] - ((x[1] - MP3[0]*sin(pi/180*x[2]) - BP3[0])^2 + (x[1] + MP3[2]*cos(pi/180*x[2]) - BP3[2])^2);
//  return t;
//  }
//
//  float fprime(float x){
//    return 2*(x-1);
//  }
//}
//
//
//void Newton_Raphson() {
//delay(100);
//if (abs(err)>1e-5){
//  Seral.println(x_iters[ctr]);
//  float x_current = x_iters[ctr];
//  float x_next = x_current - f(x_current)/fprime(x_current);
//  x_iters[ctr+1]=x_next;
//  ctr++;
//  err= F(x_next);
//  }
//}
//
//};
//#endif
