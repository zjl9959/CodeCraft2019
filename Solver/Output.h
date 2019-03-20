#pragma once
#ifndef CODE_CRAFT_2019_OUTPUT_H
#define CODE_CRAFT_2019_OUTPUT_H
/*
 * ��Solutionת��Ϊ�ٷ��涨�������ʽ��
 * ��������浽ָ��·�����ļ��С�
 */
#include "Instance.h"

namespace codecraft2019 {

struct Routine {
    ID car_id;
    Time start_time;
    std::vector<ID> roads;
	std::vector<ID> crosses;//��roadsת��������·��
    Routine() {};
    Routine(const Routine &rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads = rhs.roads;
        crosses = rhs.crosses;
    }
    Routine(Routine &&rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads.swap(rhs.roads);
        crosses.swap(rhs.crosses);
    }

    Routine& operator = (const Routine &rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads = rhs.roads;
        crosses = rhs.crosses;
        return *this;
    }
    Routine& operator = (Routine &&rhs) {
        car_id = rhs.car_id;
        start_time = rhs.start_time;
        roads.swap(rhs.roads);
        crosses.swap(rhs.crosses);
        return *this;
    }
};

struct Output {
    Output(Instance* instance): ins_(instance) {};
    ~Output() { ins_ = nullptr; }
    bool save(Environment &env);
    Instance* ins_;
    std::vector<Routine> routines;
};

}

#endif // !CODE_CRAFT_2019_OUTPUT_H
