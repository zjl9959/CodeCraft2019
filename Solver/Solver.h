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
struct RAux {
	RawCar *raw_car;
	ID car_id;
	Time cost_time;
	bool **subPath; //该车辆的最短路包含的子路径
	RAux(int cross_size, Time cost_time, RawCar *raw_car, ID car_id) :
		cost_time(cost_time) ,raw_car(raw_car),car_id(car_id) {
		subPath = new bool*[cross_size]();
		for (int i = 0; i < cross_size; ++i) {
			subPath[i] = new bool[cross_size]();
		}
	}
	RAux() {}
};
struct PathInfo {
	ID from;
	ID to;
	int num; // 数量
};
struct RoadCondition {
	int vacancy_num;//空位数
	double avg_speed_ratio; //道路上的车辆的实际行驶速度与最大速度的比值的平均值
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
		subPathInfo.resize(cross_size);
		for (int i = 0; i < cross_size; ++i) {
			subPathInfo[i].resize(cross_size);
		}
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
	void init_solution_once();
	void init_solution_2();
    void binary_generate_solution();
	void generate_futher_solution();
	Time handle_deadLock();
	void start_early(List<Routine> &temp_routines, int d1, int d2);
	void insert_to_early(List<Routine>& temp_routines);
	void local_search();
	void find_newPath_and_time(std::vector<Routine> &temp_routines, List<std::pair<ID, int>> &neighbour,Neighbour neigh,int K=10, Time my_min=350, Time my_max=INT_MAX);
	Time priority_first_search(Time start_time, Car *car, List<ID> &roads);
	Time changeTime(int total_car_num, int car_num_mid, std::vector<std::pair<Time, ID>> &run_time, std::vector<Time> &start_times);
	int check_solution(const std::vector<Routine> &routines, Aux &aux,Mode mode = CheckAll);
	void get_routines_cost_time();
    void test_treeSearch();
	
	Time find_best_start_time(Car *car, Routine &routine, std::vector<ID> &best_roads);
	/* 调度相关 */
	void driveAllCarJustOnRoadToEndState(Road *road);
	void driveCarOnChannelToEndState(Road *road, int ch);
	void driveCarInGarage();
	void recordProbOutCross(CarLocationOnRoad *carL,Cross *cross,Road *road, const std::vector<Routine> &routines);//记录可能会出路口的CarL
	void clearRoadVector(int size);
	Road* getNextRoad(CarLocationOnRoad * carL, Cross *cross, const std::vector<Routine> &routines);//根据当前路口和carL获取下一条道路的指针
	bool moveToNextRoad(Road *road,Road *next_road,CarLocationOnRoad *carL, const std::vector<Routine> &routines);
public:
    TimeSlice timeslice;
private:
    /*辅助计算*/
    Time min_time_cost(const ID car, const ID from, const ID to) const;
	std::vector<CarLocationOnRoad *> carL_inDst;//保存到达终点的carL指针
protected:
    Instance* ins_;
	Time current_time;
	int t;
	Time total_time;
    Output* output_;
    Environment* env_;
    Configure* cfg_;
private:
	Topo topo; 
	Aux aux;
	std::vector<RAux> rauxs;
    List<List<List<ID>>> shortest_paths; // 任意两点之间的最短路径
	List<List<List<ID>>> shortest_cross_paths; // 任意两点间最短路径经过的路口
	List<List<List<ID>>> other_paths;
	List<List<List<RoadCondition>>> time_road_condition; //每个时间片的路况
	List<List<List<ID>>> subPathInfo;//[道路跳数]
	List<std::pair<ID, int>> time_diff; //每辆车的实际出发时间与计划时间的时间差
	List<std::pair<ID,int>> cars_cost_time; //每辆车的消耗时间
	List<std::pair<ID, ID>> zero_speed_car; //
	List<std::pair<ID,ID>> dead_lockCar; // car_id, road_id
	void read_from_file();
	int car_size ;
	int road_size ;
	int double_road_size;
	int cross_size;
	int inDst_num;
	int cars_totalTime;
	int latest_real_start_time;
	int latest_start_time;
	int randseed;
};

}

#endif // !CODE_CRAFT_2019_SOLVER_H
