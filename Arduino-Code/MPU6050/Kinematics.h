#ifndef _Kinematics_h
#define _Kinematics_h
#include <MatrixMath.h>
#define pi 3.14
//double p[3][1];
double length4, length5, length6;
class Kinematics{
  public:
    //Kinematics(){
    //}
  double p[3][1];  
  double Base_radius,Manipulator_radius;  
  //double BP1[3],BP2[3],BP3[3];
  //double MP1[3],MP2[3],MP3[3];
  double a1[3][3], a2[3][3], a3[3][3];
  double mul1[3], mul2[3], mul3[3];
  double sub1[3], sub2[3], sub3[3], sub4[3], sub5[3], sub6[3];
  double R_MP[3][3], R_MP1[3][3];
  double c1[3] ,c2[3], c3[3];
  double ans1[3], ans2[3], ans3[3],ans11[3], ans22[3], ans33[3];
  double matC[9],matA[9][3];
  double transA[3][9],A_transA[3][3],cBar[3][1], inv_ans[3][3];
  double length1[3], length2[3], length3[3];
   

  double pose[3], R_tar[3][3], roll_tar, pitch_tar, yaw_tar, res_1[3], res_2[3], res_3[3], res_4[3], res_5[3], res_6[3], len_1[3], len_2[3], len_3[3], TarLen_1, TarLen_2, TarLen_3;
  double J[3][3], J_inv[3][3], J_norm, J_inv_norm, cond_num;
 double BP1[3] = {40, 0, 20};
 double BP2[3] = {340,0,0};
 double BP3[3] = {0,0,300};

 double MP1[3] = {0, 0, 0};
 double MP2[3] = {60,0,0};
 double MP3[3] = {0,0,60};
   
   
  void calculateKinematics(double direc_vec1[], double direc_vec2[], double direc_vec3[], double rollMP, double pitchMP) {
//Serial.println(rollMP*180/pi);
//    BP1[0] = 40;     BP1[1] = 0;        BP1[2] = 20;
//    BP2[0]= 340;     BP2[1] = 0;        BP2[2] = 0;
//    BP3[0]= 0;      BP3[1] = 0;        BP3[2] = 300; 
//
//    MP1[0] = 0;      MP1[1]= 0;         MP1[2]=0;
//    MP2[0] = 60;     MP2[1]= 0;         MP2[2]=0;
//    MP3[0] =0;       MP3[1]= 0;         MP3[2]=60;
    
    
    a1[0][0] = matA[0][0] = 0.0;                a1[0][1] = matA[0][1] = -direc_vec1[2];             a1[0][2] = matA[0][2] = direc_vec1[1];
    a1[1][0] = matA[1][0] = direc_vec1[2];      a1[1][1] = matA[1][1] = 0.0;                        a1[1][2] = matA[1][2] = -direc_vec1[0];
    a1[2][0] = matA[2][0] = -direc_vec1[1];     a1[2][1] = matA[2][1] = direc_vec1[0];              a1[2][2] = matA[2][2] = 0.0;

    a2[0][0] = matA[3][0] = 0.0;                a2[0][1] = matA[3][1] = -direc_vec2[2];             a2[0][2] = matA[3][2] = direc_vec2[1];
    a2[1][0] = matA[4][0] = direc_vec2[2];      a2[1][1] = matA[4][1] = 0.0;                        a2[1][2] = matA[4][2] = -direc_vec2[0];
    a2[2][0] = matA[5][0] = -direc_vec2[1];     a2[2][1] = matA[5][1] = direc_vec2[0];              a2[2][2] = matA[5][2] = 0.0;

    a3[0][0] = matA[6][0] = 0.0;                a3[0][1] = matA[6][1] = -direc_vec3[2];             a3[0][2] = matA[6][2] = direc_vec3[1];
    a3[1][0] = matA[7][0] = direc_vec3[2];      a3[1][1] = matA[7][1] = 0.0;                        a3[1][2] = matA[7][2] = -direc_vec3[0];
    a3[2][0] = matA[8][0] = -direc_vec3[1];     a3[2][1] = matA[8][1] = direc_vec3[0];              a3[2][2] = matA[8][2] = 0.0;
    
//Matrix.Print((double*)a1, 3, 3, "a1"); 
//Matrix.Print((double*)a2, 3, 3, "a2");
//Matrix.Print((double*)a3, 3, 3, "a3");
//Matrix.Print((double*)matA, 9, 3, "A");
// 
  
 // R_MP[0][0] = 1; R_MP[0][1] = 0; R_MP[0][2] = 0;
 // R_MP[1][0] =0; R_MP[1][1] = 1; R_MP[1][2] = 0;
//  R_MP[2][0] =0 ; R_MP[2][1] =0 ; R_MP[2][2] = 1;

double roll_MP = 0.0;
double pitch_MP = -(rollMP-90)*pi/180;
double yaw_MP = 0.0;
//Serial.println(pitch_MP*180/pi);

  
R_MP[0][0] = cos(pitch_MP)*cos(yaw_MP);
R_MP[0][1] = -cos(pitch_MP)*sin(yaw_MP);
R_MP[0][2] = sin(pitch_MP);
R_MP[1][0] = cos(roll_MP)*sin(yaw_MP)+sin(roll_MP)*sin(pitch_MP)*cos(yaw_MP);
R_MP[1][1] = cos(roll_MP)*cos(yaw_MP)-sin(roll_MP)*sin(pitch_MP)*sin(yaw_MP);
R_MP[1][2] = -sin(roll_MP)*cos(pitch_MP);
R_MP[2][0] = sin(roll_MP)*sin(yaw_MP)-cos(roll_MP)*sin(pitch_MP)*cos(yaw_MP);
R_MP[2][1] = sin(roll_MP)*cos(yaw_MP)+cos(roll_MP)*sin(pitch_MP)*sin(yaw_MP) ;
R_MP[2][2] = cos(roll_MP)*cos(pitch_MP) ;

//double roll_MP1 = 0;
//double pitch_MP1=pi/180*90;
//double yaw_MP1=-pi/180*90;

//R_MP1[0][0] = cos(pitch_MP1)*cos(yaw_MP1);
//R_MP1[0][1] = -cos(pitch_MP1)*sin(yaw_MP1);
//R_MP1[0][2] = sin(pitch_MP1);
//R_MP1[1][0] = cos(roll_MP1)*sin(yaw_MP1)+sin(roll_MP1)*sin(pitch_MP1)*cos(yaw_MP1);
//R_MP1[1][1] = cos(roll_MP1)*cos(yaw_MP1)-sin(roll_MP1)*sin(pitch_MP1)*sin(yaw_MP1);
//R_MP1[1][2] = -sin(roll_MP1)*cos(pitch_MP1);
//R_MP1[2][0] = sin(roll_MP1)*sin(yaw_MP1)-cos(roll_MP1)*sin(pitch_MP1)*cos(yaw_MP1);
//R_MP1[2][1] = sin(roll_MP1)*cos(yaw_MP1)+cos(roll_MP1)*sin(pitch_MP1)*sin(yaw_MP1) ;
//R_MP1[2][2] = cos(roll_MP1)*cos(pitch_MP1) ;

 //Matrix.Print((double*)R_MP, 3, 3, "R_MP");
  Matrix.Multiply((double*)R_MP, (double*)MP1, 3,3,1, (double*)ans1);
  //Matrix.Multiply((double*)ans1, (double*)MP1, 3,3,1, (double*)ans11);\

           Matrix.Multiply((double*)R_MP, (double*)MP2, 3,3,1, (double*)ans2);
           //Matrix.Multiply((double*)ans2, (double*)MP2, 3,3,1, (double*)ans22);

                      Matrix.Multiply((double*)R_MP, (double*)MP3, 3,3,1, (double*)ans3);  
                    //Matrix.Multiply((double*)ans3, (double*)MP3, 3,3,1, (double*)ans33);
  

  Matrix.Subtract((double*)BP1, (double*)ans1, 3,1, (double*)sub1);
  Matrix.Subtract((double*)BP2, (double*)ans2, 3,1, (double*)sub2);
  Matrix.Subtract((double*)BP3, (double*)ans3, 3,1, (double*)sub3);

  Matrix.Multiply((double*)a1, (double*)sub1, 3,3,1, (double*)c1);
  Matrix.Multiply((double*)a2, (double*)sub2, 3,3,1, (double*)c2);
  Matrix.Multiply((double*)a3, (double*)sub3, 3,3,1, (double*)c3);
  
  matC[0] = c1[0]; matC[1] = c1[1]; matC[2] = c1[2];
  matC[3] = c2[0]; matC[4] = c2[1]; matC[5] = c2[2];
  matC[6] = c3[0]; matC[7] = c3[1]; matC[8] = c3[2];
  
  //Matrix.Print((double*)matC, 9, 1, "C");
  
  Matrix.Transpose((double*)matA, 9, 3, (double*)transA);
  Matrix.Multiply((double*)transA, (double*)matA, 3, 9, 3, (double*)A_transA);
   //Matrix.Print((double*)A_transA, 3,3, "A'*A");
  Matrix.Invert((double*)A_transA, 3);
  Matrix.Multiply((double*)transA, (double*)matC, 3, 9, 1, (double*)cBar);
  Matrix.Multiply((double*)A_transA, (double*)cBar, 3, 3, 1, (double*)p);

//  Matrix.Print((double*)transA, 3,9, "A'");
// 
//  Matrix.Print((double*)cBar, 3,1, "cBar");
  //Matrix.Print((double*)p, 3,1, "Position");

  
  Matrix.Multiply((double*)R_MP, (double*)MP1, 3,3,1, (double*)mul1); 
  Matrix.Multiply((double*)R_MP, (double*)MP2, 3,3,1, (double*)mul2);
  Matrix.Multiply((double*)R_MP, (double*)MP3, 3,3,1, (double*)mul3);
  Matrix.Subtract((double*)mul1, (double*)BP1, 3, 1, (double*)sub4);
  Matrix.Subtract((double*)mul2, (double*)BP2, 3, 1, (double*)sub5);
  Matrix.Subtract((double*)mul3, (double*)BP3, 3, 1, (double*)sub6);
  Matrix.Add((double*)p, (double*)sub4, 3, 1, (double*)length1);
  Matrix.Add((double*)p, (double*)sub5, 3, 1, (double*)length2);
  Matrix.Add((double*)p, (double*)sub6, 3, 1, (double*)length3);
//  Matrix.Print((double*)length1, 3,1, "Length 1");
//  Matrix.Print((double*)length2, 3,1, "Length 2");
//   Matrix.Print((double*)length3, 3,1, "Length 3");
  length4 = sqrt(sq(length1[0])+sq(length1[1])+sq(length1[2]));
  length5 = sqrt(sq(length2[0])+sq(length2[1])+sq(length2[2]));
  length6 = sqrt(sq(length3[0])+sq(length3[1])+sq(length3[2]));  

  //Matrix.println((double*)length1);
 //Serial.print("Kinematic Lengths");Serial.print("\t"); Serial.print(length4);Serial.print("\t");  Serial.print(length5);Serial.print("\t");  Serial.println(length6);
// Serial.println();
  //Serial.println(length6);
};
  
void calTargetPose(double x, double z, double phi){
// BP1[0] = 40;     BP1[1] = 0;        BP1[2] = 20;
//    BP2[0]= 340;     BP2[1] = 0;        BP2[2] = 0;
//    BP3[0]= 0;      BP3[1] = 0;        BP3[2] = 300;
//
//    MP1[0] = 0;      MP1[1]= 0;         MP1[2]=0;
//    MP2[0] = 60;     MP2[1]= 0;         MP2[2]=0;
//    MP3[0] =0;       MP3[1]= 0;         MP3[2]=60;
  
roll_tar = phi*pi/180;
pitch_tar = 0.0;
yaw_tar = 0.0;

R_tar[0][0] = cos(pitch_tar)*cos(yaw_tar);
R_tar[0][1] = -cos(pitch_tar)*sin(yaw_tar);
R_tar[0][2] = sin(pitch_tar);
R_tar[1][0] = cos(roll_tar)*sin(yaw_tar)+sin(roll_tar)*sin(pitch_tar)*cos(yaw_tar);
R_tar[1][1] = cos(roll_tar)*cos(yaw_tar)-sin(roll_tar)*sin(pitch_tar)*sin(yaw_tar);
R_tar[1][2] = -sin(roll_tar)*cos(pitch_tar);
R_tar[2][0] = sin(roll_tar)*sin(yaw_tar)-cos(roll_tar)*sin(pitch_tar)*cos(yaw_tar);
R_tar[2][1] = sin(roll_tar)*cos(yaw_tar)+cos(roll_tar)*sin(pitch_tar)*sin(yaw_tar) ;
R_tar[2][2] = cos(roll_tar)*cos(pitch_tar) ;

pose[0] = x;
pose[1] = 0.0;
pose[2] = z;

Matrix.Multiply((double*)R_tar, (double*)MP1, 3,3,1, (double*)res_1); 
Matrix.Multiply((double*)R_tar, (double*)MP2, 3,3,1, (double*)res_2);
Matrix.Multiply((double*)R_tar, (double*)MP3, 3,3,1, (double*)res_3);
Matrix.Subtract((double*)res_1, (double*)BP1, 3, 1, (double*)res_4);
Matrix.Subtract((double*)res_2, (double*)BP2, 3, 1, (double*)res_5);
Matrix.Subtract((double*)res_3, (double*)BP3, 3, 1, (double*)res_6);

Matrix.Add((double*)pose, (double*)res_4, 3, 1, (double*)len_1);
Matrix.Add((double*)pose, (double*)res_5, 3, 1, (double*)len_2);
Matrix.Add((double*)pose, (double*)res_6, 3, 1, (double*)len_3);

TarLen_1 = sqrt(sq(len_1[0])+sq(len_1[1])+sq(len_1[2]));
TarLen_2 = sqrt(sq(len_2[0])+sq(len_2[1])+sq(len_2[2]));
TarLen_3 = sqrt(sq(len_3[0])+sq(len_3[1])+sq(len_3[2]));

double den_1 =  (2*sqrt(sq(x - BP1[0] + MP1[0]*cos(phi*pi/180) + MP1[2]*sin(phi*pi/180)) + sq(z - BP1[2] + MP1[2]*cos(phi*pi/180) - MP1[0]*sin(phi*pi/180))));
double den_2 =  (2*sqrt(sq(x - BP2[0] + MP2[0]*cos(phi*pi/180) + MP2[2]*sin(phi*pi/180)) + sq(z - BP2[2] + MP2[2]*cos(phi*pi/180) - MP2[0]*sin(phi*pi/180))));
double den_3 =  (2*sqrt(sq(x - BP3[0] + MP3[0]*cos(phi*pi/180) + MP3[2]*sin(phi*pi/180)) + sq(z - BP3[2] + MP3[2]*cos(phi*pi/180) - MP3[0]*sin(phi*pi/180))));
//Serial.println(den_1);
//Serial.println(den_2);
//Serial.println(den_3);
 
J[0][0] = (2*x - 2*BP1[0] + 2*MP1[0]*cos(phi*pi/180) + 2*(MP1[2]*sin(phi*pi/180))) / den_1;
J[0][1] = (2*z - 2*BP1[2] + 2*MP1[2]*cos(phi*pi/180) - 2*(MP1[0]*sin(phi*pi/180))) / den_1;
J[0][2] = -(2*(MP1[2]*cos(phi*pi/180) - MP1[0]*sin(phi*pi/180)) * (x - BP1[0] + MP1[0]*cos(phi*pi/180) + MP1[2]*sin(phi*pi/180)) - 2*(MP1[0]*cos(phi*pi/180) + MP1[2]*sin(phi*pi/180)) * (z - BP1[2] + MP1[2]*cos(phi*pi/180) - MP1[0]*sin(phi*pi/180))) / den_1;

J[1][0] = (2*x - 2*BP2[0] + 2*MP2[0]*cos(phi*pi/180) + 2*(MP2[2] * sin(phi*pi/180))) / den_2;    
J[1][1] = (2*z - 2*BP2[2] + 2*MP2[2]*cos(phi*pi/180) - 2*(MP2[0]*sin(phi*pi/180))) / den_2;
J[1][2] = -(2*(MP2[2]*cos(phi*pi/180) - MP2[0]*sin(phi*pi/180)) * (x - BP2[0] + MP2[0]*cos(phi*pi/180) + MP2[2]*sin(phi*pi/180)) - 2*(MP2[0]*cos(phi*pi/180) + MP2[2]*sin(phi*pi/180)) * (z - BP2[2] + MP2[2]*cos(phi*pi/180) - MP2[0]*sin(phi*pi/180))) / den_2;

J[2][0] = (2*x - 2*BP3[0] + 2*MP3[0]*cos(phi*pi/180) + 2*(MP3[2] * sin(phi*pi/180))) / den_3;
J[2][1] = (2*z - 2*BP3[2] + 2*MP3[2]*cos(phi*pi/180) - 2*(MP3[0] * sin(phi*pi/180))) / den_3;
J[2][2] = -(2*(MP3[2]*cos(phi*pi/180) - MP3[0]*sin(phi*pi/180)) * (x - BP3[0] + MP3[0]*cos(phi*pi/180) + MP3[2]*sin(phi*pi/180)) - 2*(MP3[0]*cos(phi*pi/180) + MP3[2]*sin(phi*pi/180)) * (z - BP3[2] + MP3[2]*cos(phi*pi/180) - MP3[0]*sin(phi*pi/180))) / den_3;

J_norm = sqrt(sq(J[0][0]) + sq(J[0][1]) + sq(J[0][2]) + sq(J[1][0]) + sq(J[1][1]) + sq(J[1][2]) + sq(J[2][0]) + sq(J[2][1]) + sq(J[2][2]));
//Matrix.Print((double*)J, 3,3, "J");
Matrix.Invert((double*)J, 3);
J_inv_norm =  sqrt(sq(J[0][0]) + sq(J[0][1]) + sq(J[0][2]) + sq(J[1][0]) + sq(J[1][1]) + sq(J[1][2]) + sq(J[2][0]) + sq(J[2][1]) + sq(J[2][2]));
cond_num = J_norm*J_inv_norm;
//Matrix.println((double*)J_inv_norm, 3,3, "J Norm Inv");
//Serial.print("cond_num");Serial.print("\t"); Serial.println(cond_num);
 
//
//Serial.print("Target Lengths_kin");Serial.print("\t");
//Serial.print(TarLen_1);Serial.print("\t");
//        Serial.print(TarLen_2);Serial.print("\t");
//        Serial.println(TarLen_3);
//Matrix.Print((double*)pose, 3,1, "pose");
//Matrix.Print((double*)R_tar,3,3, "Rotation");
//
//  Matrix.Print((double*)MP1, 3,1, "MP1");
//  Matrix.Print((double*)MP2, 3,1, "MP2");
//   Matrix.Print((double*)MP3, 3,1, "MP3");
//
//  Matrix.Print((double*)res_1, 3,1, "res_1");
//  Matrix.Print((double*)res_2, 3,1, "res_2");
//   Matrix.Print((double*)res_3, 3,1, "res_3");
//
//   
//  Matrix.Print((double*)res_4, 3,1, "res_4");
//  Matrix.Print((double*)res_5, 3,1, "res_5");
//   Matrix.Print((double*)res_6, 3,1, "res_6");
};
};
#endif
