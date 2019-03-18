#pragma once
#ifndef CODE_CRAFT_2019_SOLVER_H
#define CODE_CRAFT_2019_SOLVER_H
/*
 * 求解的核心算法。
 */

#include "Output.h"
#include "Solution.h"
#include "Topo.h"
namespace codecraft2019 {

class Solver {
public:
    Solver(Instance *ins, Output *output, Environment *env, Configure *cfg) :
        ins_(ins), output_(output), env_(env), cfg_(cfg) ,topo(ins){
	};
    ~Solver() {
        ins_ = nullptr;
        output_ = nullptr;
        env_ = nullptr;
        cfg_ = nullptr;
    };
    void testIO();	
	void init_solution();
	void check_solution();
protected:
    Instance* ins_;
	Time total_time;
    Output* output_;
    Environment* env_;
    Configure* cfg_;
public:
	Topo topo; 
    // [TODO]添加算法用到的数据结构
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
