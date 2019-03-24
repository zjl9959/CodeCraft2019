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
struct RAux {
	RawCar *raw_car;
	ID car_id;
	Time cost_time;
	Time start_time;
};

class Solver {
public:
    Solver(Instance *ins, Output *output, Environment *env, Configure *cfg) :
        ins_(ins), output_(output), env_(env), cfg_(cfg) ,topo(ins),
		aux(ins_->raw_crosses.size(),ins_->raw_roads.size()){
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
	void init_solution_2();
    void binary_generate_solution();
	void generate_futher_solution();
	Time changeTime(int total_car_num, int car_num_mid, std::vector<std::pair<Time, ID>> &run_time);
	int check_solution(const std::vector<Routine> &routines, Aux &aux);

	/* ������� */
	void driveAllCarJustOnRoadToEndState(Road *road);
	void driveCarOnChannelToEndState(Road *road, int ch);
	void driveCarInGarage();
	void recordProbOutCross(CarLocationOnRoad *carL,Cross *cross,Road *road, const std::vector<Routine> &routines);//��¼���ܻ��·�ڵ�CarL
	void clearRoadVector(int size);
	Road* getNextRoad(CarLocationOnRoad * carL, Cross *cross, const std::vector<Routine> &routines);//���ݵ�ǰ·�ں�carL��ȡ��һ����·��ָ��
	bool moveToNextRoad(Road *road,Road *next_road,CarLocationOnRoad *carL);

private:
    /*��������*/
    Time min_time_cost(const ID car, const ID from, const ID to) const;
	std::vector<Routine> temp_routines;//�����м��
	std::vector<CarLocationOnRoad *> carL_inDst;//���浽���յ��carLָ��
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
	vector<RAux> rauxs;
	TimeSlice timeslice;
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
