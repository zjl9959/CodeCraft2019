#pragma once
#ifndef CODE_CRAFT_2019_COMMON_H
#define CODE_CRAFT_2019_COMMON_H

#include <string>

namespace codecraft2019 {

/*******�����������������********/
#define INVALID_ID -1           // ��Ч��ID

using ID = int;
using Length = int;
using Speed = int;
using Time = int;
using Channel = int;
enum Turn { Left, Right, Front, Invalid };

/***********�����������·������***************/
struct Environment {
    std::string car_path;       // ��������car.txt·����
    std::string road_path;      // ��������road.txt·����
    std::string cross_path;     // ��������cross.txt·����
    std::string answer_path;    // ������answer.txt·����
    Environment(char* Car_path, char* Road_path, char* Cross_path, char* Answer_path) :
        car_path(Car_path), road_path(Road_path), cross_path(Cross_path), answer_path(Answer_path) {};
};

/**********  �����㷨��������*****************/
struct Configure {
    // [TODO]�����������ò�����
    double maxseconds = 10.0;           // �������ʱ�䣬��λ�롣
    double maxmilliseconds = 1000.0;    // �������ʱ�䣬��λ���롣
    size_t maxiter = 1 << 31;           // ������������
};

}

#endif // !CODE_CRAFT_2019_COMMON_H