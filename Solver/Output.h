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
};

struct Output {
    Output(Instance* instance): ins_(instance) {};
    ~Output() { ins_ = nullptr; }
    bool save(Environment &env);
    Instance* ins_;
    std::vector<Routine *> routines;
};

}

#endif // !CODE_CRAFT_2019_OUTPUT_H
