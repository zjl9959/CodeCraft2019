#pragma once
#ifndef CODE_CRAFT_2019_SOLVER_H
#define CODE_CRAFT_2019_SOLVER_H
/*
 * ���ĺ����㷨��
 */

#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

class Solver {
public:
    Solver(Instance *ins, Output *output, Environment *env, Configure *cfg) :
        ins_(ins), output_(output), env_(env), cfg_(cfg) {};
    ~Solver() {
        ins_ = nullptr;
        output_ = nullptr;
        env_ = nullptr;
        cfg_ = nullptr;
    };
    void testIO();
protected:
    Instance* ins_;
    Output* output_;
    Environment* env_;
    Configure* cfg_;
    // [TODO]����㷨�õ������ݽṹ
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
