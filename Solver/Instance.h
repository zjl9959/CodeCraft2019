#pragma once
#ifndef CODE_CRAFT_2019_INSTANCE_H
#define CODE_CRAFT_2019_INSTANCE_H
/*
 * ���������ĵ����������ṹ��
 * �������ĵ��м����������ݡ�
 */
#include <vector>

#include "Common.h"

namespace codecraft2019 {

struct Road {
    ID id;                  // ��·ID
    Length length;          // ��·����
    Speed speed_limit;      // ��·����
    int road_num;           // ÿ���򳵵���Ŀ
    ID begin;               // ���ID
    ID end;                 // �յ�ID
    bool bidirection;       // �Ƿ�Ϊ˫�򳵵�
};

struct Cross {
    ID id;                  // ����·��ID
    ID north;               // ��/�Ϸ�·��
    ID east;                // ��/�ҷ�·��
    ID west;                // ��/��·��
    ID south;               // ��/�·�·��
};

struct Car {
    ID id;                  // ����ID
    ID origin;              // ���ID
    ID destination;         // �յ�ID
    Speed max_speed;        // �����ʻ�ٶ�
    Time go_time;           // �ƻ�����ʱ��
};

struct Instance {
    Instance();
    void load(Environment &env);
    ~Instance();

    std::vector<Road> roads;
    std::vector<Car> cars;
    std::vector<Cross> crosses;
};

}

#endif // !CODE_CRAFT_2019_INSTANCE_H
