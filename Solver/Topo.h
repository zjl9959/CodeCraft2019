#pragma once
#ifndef CODE_CRAFT_2019_TOPO_H
#define CODE_CRAFT_2019_TOPO_H
#include "Output.h"
#include "Solution.h"

namespace codecraft2019 {

struct node {
	ID id;
	Length dist;
	node(ID id,Length dist):id(id), dist(dist){}
	bool operator<(const node k)const
	{
		return dist > k.dist;
	}
};
struct CarLocationOnRoad {
	ID car_id;
	STATE state;//车的状态
	int location;//车在道路中的位置
	int index;//当前道路在路径中的index
	int indexInRoutine; //该车属于哪一个routine
	ID channel_id;
	Turn turn;
	Time start_time;
	int real_speed; //车辆的实际行驶速度
};
typedef struct Cross Cross;
typedef struct Road Road;

struct Cross {
	RawCross *raw_cross;
	std::vector<Road *> road;//出路口方向的路
	std::vector<Road *> inCrossRoad;//入路口方向的路
	Cross(RawCross *raw_cross) {
		this->raw_cross = raw_cross;
	}
};
struct Road {
	RawRoad *raw_road;
	Cross *from, *to;
	ID from_id, to_id;
	int vacancy_thresh; // 空位数阈值
	std::vector<std::vector<CarLocationOnRoad *>> channel_carL;
	std::vector<CarLocationOnRoad *> waitOutCarL;
	std::vector<CarLocationOnRoad *> willOnRoad;
	Road(RawRoad *raw_road,Cross *from,Cross *to):raw_road(raw_road),from(from),to(to) {
		from_id = from->raw_cross->id;
		to_id = to->raw_cross->id;
		vacancy_thresh = (raw_road->channel*raw_road->length*1.0) / 2.0;
	}

};

struct Car{
	RawCar *raw_car;
	Cross *from, *to;
	Car(RawCar *raw_car) {
		this->raw_car = raw_car;
	}
};
struct InterRoutine {
	Car *car;
	Time run_time;
	InterRoutine(Car *car, Time run_time) :car(car), run_time(run_time) {}
};
struct Aux {
	std::vector<std::vector<InterRoutine *>> **car_same;//出发点和起点及计划出发时间都相同的车辆
	//std::vector<std::vector<std::vector<int>>> roadNullNumOfTime; //道路i->j 随时间的空位数
	//std::vector<std::vector<int>> type_numTime; //某一时刻达到终点的某一类车的数目
	//int type_num;
	int real_car_num; // 实际参与调度的车辆数
	Aux(int cross_size,int road_size) {
		car_same = new std::vector<std::vector<InterRoutine *>>*[cross_size];
		for (int i = 0; i < cross_size; i++) {
			car_same[i] = new std::vector<std::vector<InterRoutine *>>[cross_size];
		}
		for (int i = 0; i < cross_size; i++) {
			for (int j = 0; j < cross_size; ++j) {
				car_same[i][j].resize(LATEST_PLAN_TIME);
			}
		}
	}
};
class Topo {
public:
	Topo(Instance *ins);
	void Floyd();
	std::vector<ID> Dijkstra(ID src, ID dst, ID tabuRoad = INVALID_ID);
	void printPath();
	void init_myTopo();
	Turn getRoadTurn(ID cross_id, ID road1, ID road2);//获取cross的两条道路的转向

	std::vector<Road *> roads;
	std::vector<Cross *> crosses;
	std::vector<Car *> cars;

	Road ***adjRoad;

	std::priority_queue<node> q;
	ID **adjRoadID;//两个路口之间的道路ID（原始道路）
	ID **pathID;//最短路经过的crossID
	Length **sPathLen;//最短路长度
	Turn **RoadTurn;
	int cross_size;
	int car_size;
	int rsize;
	std::vector<CarLocationOnRoad *> allCarLOnRoad;
protected:
	Instance *ins_;

	void clear(std::priority_queue<node>& q) {
		std::priority_queue<node> empty;
		swap(empty, q);
	}
};
}
#endif // !CODE_CRAFT_2019_TOPO_H
