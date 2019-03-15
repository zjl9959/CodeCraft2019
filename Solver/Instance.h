#pragma once
#ifndef CODE_CRAFT_2019_INSTANCE_H
#define CODE_CRAFT_2019_INSTANCE_H
/*
 * ���������ĵ����������ṹ��
 * �������ĵ��м����������ݡ�
 */
#include <vector>

#include "Common.h"
#include "Utility.h"

namespace codecraft2019 {

struct Road {
    ID id;                  // ��·ID
    Length length;          // ��·����
    Speed speed;            // ��·����
    Channel channel;        // ÿ���򳵵���Ŀ
    ID from;                // ���ID
    ID to;                  // �յ�ID
    bool is_duplex;         // �Ƿ�Ϊ˫�򳵵�
    Road(ID Id, Length Length, Speed Speed, Channel Channel, ID From, ID To, bool Is_duplex) :
        id(Id), length(Length), speed(Speed), channel(Channel), from(From), to(To), is_duplex(Is_duplex) {};
};

struct Car {
    ID id;                  // ����ID
    ID from;                // ���ID
    ID to;                  // �յ�ID
    Speed speed;            // �����ʻ�ٶ�
    Time plan_time;         // �ƻ�����ʱ��
    Car(ID Id, ID From, ID To, Speed Speed, Time Plan_time) :
        id(Id), from(From), to(To), speed(Speed), plan_time(Plan_time) {};
};

struct Cross {
    ID id;                  // ����·��ID
    ID north;               // ��/�Ϸ�·��
    ID east;                // ��/�ҷ�·��
    ID west;                // ��/��·��
    ID south;               // ��/�·�·��
    Cross(ID Id, ID North, ID East, ID West, ID South) :id(Id), north(North), east(East), south(South) {};
};

struct Instance {
    Instance() {};
    Instance(Environment &env) { load(env); };
    ~Instance() {};

    bool load(Environment &env);    // ������·���м���������

    bool valid = true;    // ָʾ���ص������Ƿ�Ϸ���

    std::vector<Road> roads;
    std::vector<Car> cars;
    std::vector<Cross> crosses;
	ID changeToZeroID(ID src,ID deta);
    ZeroBasedConsecutiveIdMap<ID, ID> road_map;
    ZeroBasedConsecutiveIdMap<ID, ID> car_map;
    ZeroBasedConsecutiveIdMap<ID, ID> cross_map;
};

}

#endif // !CODE_CRAFT_2019_INSTANCE_H
