#pragma once
#ifndef CODE_CRAFT_2019_SOLVER_H
#define CODE_CRAFT_2019_SOLVER_H
/*
 * ���ĺ����㷨��
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

	/* ������� */
	void driveAllCarJustOnRoadToEndState(Road *road);
	void driveCarOnChannelToEndState(Road *road, int ch);
	void driveCarInGarage();
	void recordProbOutCross(CarLocationOnRoad *carL,Cross *cross,Road *road);//��¼���ܻ��·�ڵ�CarL
	Road* getNextRoad(CarLocationOnRoad * carL, Cross *cross);//���ݵ�ǰ·�ں�carL��ȡ��һ����·��ָ��
	bool moveToNextRoad(Road *road,Road *next_road,CarLocationOnRoad *carL);

private:
    /*��������*/
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
    List<List<List<ID>>> shortest_paths; // ��������֮������·��
	void read_from_file();
	int car_size ;
	int road_size ;
	int double_road_size;
	int cross_size;
	int inDst_num;
	int cars_totalTime;
    // [TODO]����㷨�õ������ݽṹ
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
