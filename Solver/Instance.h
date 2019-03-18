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

struct RawRoad {
    ID id;                  // ��·ID
    Length length;          // ��·����
    Speed speed;            // ��·����
    Channel channel;        // ÿ���򳵵���Ŀ
    ID from;                // ���ID
    ID to;                  // �յ�ID
    bool is_duplex;         // �Ƿ�Ϊ˫�򳵵�
    RawRoad(ID Id, Length Length, Speed Speed, Channel Channel, ID From, ID To, bool Is_duplex) :
        id(Id), length(Length), speed(Speed), channel(Channel), from(From), to(To), is_duplex(Is_duplex) {};
};

struct RawCar {
    ID id;                  // ����ID
    ID from;                // ���ID
    ID to;                  // �յ�ID
    Speed speed;            // �����ʻ�ٶ�
    Time plan_time;         // �ƻ�����ʱ��
    RawCar(ID Id, ID From, ID To, Speed Speed, Time Plan_time) :
        id(Id), from(From), to(To), speed(Speed), plan_time(Plan_time) {};
};

struct RawCross {
    ID id;                  // ����·��ID
	ID road[4]; //�ֱ�Ϊ������������·
    ID north;               // ��/�Ϸ�·
    ID east;                // ��/�ҷ�·
    ID west;                // ��/��·
    ID south;               // ��/�·�·
    RawCross(ID Id, ID North, ID East, ID South, ID West) :id(Id), north(North), south(South), east(East) {
		road[0] = North;
		road[1] = East;
		road[2] = South;
		road[3] = West;

	};
};

struct Instance {
    Instance() {};
    Instance(Environment &env) { load(env); };
    ~Instance() {};

    bool load(Environment &env);    // ������·���м���������

    bool valid = true;    // ָʾ���ص������Ƿ�Ϸ���

    std::vector<RawRoad> roads;
    std::vector<RawCar> cars;
    std::vector<RawCross> crosses;
	ID changeToZeroID(ID src,ID deta);
	ID changeToOriginalID(ID src, ID deta);
    ZeroBasedConsecutiveIdMap<ID, ID> road_map;
    ZeroBasedConsecutiveIdMap<ID, ID> car_map;
    ZeroBasedConsecutiveIdMap<ID, ID> cross_map;
};

}

#endif // !CODE_CRAFT_2019_INSTANCE_H
