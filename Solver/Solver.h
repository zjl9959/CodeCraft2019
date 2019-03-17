#pragma once
#ifndef CODE_CRAFT_2019_SOLVER_H
#define CODE_CRAFT_2019_SOLVER_H
/*
 * ���ĺ����㷨��
 */

#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

struct CarLocationOnRoad {
	int car_id;
	STATE state;
	int location;
};

class Topo {
public:
	Topo(Instance *ins);
	void Floyd();
	void printPath();
	Turn getRoadTurn(ID cross_id, ID road1, ID road2);//��ȡcross��������·��ת��

	ID **adjRoadID;
	ID **pathID;//���·������crossID
	Length **sPathLen;//���·����
	Turn **RoadTurn;
	int vexnum;
	std::vector<std::vector<int>>  carsWillOnRoad;
	std::vector<std::vector<CarLocationOnRoad>> carsOnRoad;
	std::vector<std::vector<std::vector<std::vector<CarLocationOnRoad>>>> road_channel_car;
protected:
	Instance *ins_;
};
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
    // [TODO]����㷨�õ������ݽṹ
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
