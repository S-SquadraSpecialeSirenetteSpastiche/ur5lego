#ifndef LEGO_H
#define LEGO_H

#include <Eigen/Eigen>
#include "ur5lego/Pose.h" //maybe not needed
#define lego_type_num 11

//TODO: change data type usign POSe msg
typedef struct{
    _Float64 X;
    _Float64 Y;
    _Float64 Z;
    _Float64 r;
    _Float64 p;
    _Float64 y;
}position;

enum Lego{
    X1_Y1_Z2,
    X1_Y2_Z1,
    X1_Y2_Z2,
    X1_Y2_Z2_CHAMFER,
    X1_Y2_Z2_TWINFILLET,
    X1_Y3_Z2,
    X1_Y3_Z2_FILLET,
    X1_Y4_Z1,
    X1_Y4_Z2,
    X2_Y2_Z2,
    X2_Y2_Z2_FILLET,
};

const position position_list[lego_type_num] ={
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y1_Z2
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y2_Z1
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y2_Z2
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y2_Z2_CHAMFER
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y2_Z2_TWINFILLET
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y3_Z2
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y3_Z2_FILLET
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y4_Z1
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X1_Y4_Z2
    {0.43, 0.45, 5, 1.5707, -1.5707, 0},        //X2_Y2_Z2
    {0.43, 0.45, 5, 1.5707, -1.5707, 0}         //X2_Y2_Z2_FILLET
};


#endif