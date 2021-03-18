#pragma once

void InitHelper();
void SetRanges(float* inData);
void SetObjectTh(float inTh);
void ShowVFH();
float CalTargetAngle(float inX, float inY);

typedef struct stGap
{
    float angle_begin;      //  缝隙开始的角度
    float r_begin;                //  缝隙开始的障碍物距离
    float angle_end;          //  缝隙结束的角度
    float r_end;                    //  缝隙结束的障碍物距离
    float d;                             //  缝隙的可通行宽度

    int path_num;              // 可通行路线的条数
    float path_theta_1;    // 可通行路线1的方向
    float path_theta_2;    // 可通行路线2的方向
}stGap;