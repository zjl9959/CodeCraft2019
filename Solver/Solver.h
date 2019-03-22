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
        ins_(ins), output_(output), env_(env), cfg_(cfg) ,topo(ins),
		aux(ins_->raw_crosses.size()){
		car_size = ins_->raw_cars.size();
		road_size = ins_->raw_roads.size();
		double_road_size = road_size * 2;
		cross_size = ins_->raw_crosses.size();
	};
    ~Solver() {
        ins_ = nullptr;
        output_ = nullptr;
        env_ = nullptr;
        cfg_ = nullptr;
    };
    void run();
    void testIO();
    void init();
	void init_solution();
	void init_solution_once();
    void binary_generate_solution();
	int check_solution();

	/* 调度相关 */
	void driveAllCarJustOnRoadToEndState(Road *road);
	void driveCarOnChannelToEndState(Road *road, int ch);
	void driveCarInGarage();
	void recordProbOutCross(CarLocationOnRoad *carL,Cross *cross,Road *road);//记录可能会出路口的CarL
	Road* getNextRoad(CarLocationOnRoad * carL, Cross *cross);//根据当前路口和carL获取下一条道路的指针
	bool moveToNextRoad(Road *road,Road *next_road,CarLocationOnRoad *carL);

private:
    /*辅助计算*/
    Time min_time_cost(const ID car, const ID from, const ID to) const;
protected:
    Instance* ins_;
	int t;
	Time total_time;
    Output* output_;
    Environment* env_;
    Configure* cfg_;
private:
	Topo topo; 
	Aux aux;
    List<List<List<ID>>> shortest_paths; // 任意两点之间的最短路径
	void read_from_file();
	int car_size ;
	int road_size ;
	int double_road_size;
	int cross_size;
	int inDst_num;
	int cars_totalTime;
    // [TODO]添加算法用到的数据结构
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
