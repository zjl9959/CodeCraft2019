#include "Solver.h"

using namespace std;

namespace codecraft2019 {
Turn cross_roadTurn[MAX_CROSS_ROAD_NUM][MAX_CROSS_ROAD_NUM] = {
{InvalidTurn,Left,Front,Right},
{Right,InvalidTurn,Left,Front},
{Front,Right,InvalidTurn,Left},
{Left,Front,Right,InvalidTurn}
};
// 此函数用于测试IO的ID映射是否正确，与求解算法无关。
void Solver::testIO() {
    output_->routines.resize(ins_->cars.size());
    for (int i = 0; i < ins_->cars.size(); ++i) {
        output_->routines[i].car_id = ins_->cars[i].id;
        output_->routines[i].start_time = ins_->cars[i].plan_time;
        output_->routines[i].roads.push_back(ins_->cars[i].from);
        output_->routines[i].roads.push_back(ins_->cars[i].to);
    }
}

void Solver::init_solution()
{
	int car_size = ins_->cars.size();
	ID temp,next;
	Time temp_time;
	Speed speed;
	//vector<Car> notPlanCar;
	total_time = 0;
	for (auto i = 0; i < car_size; ++i) {
		Routine routine;
		Car car = ins_->cars[i];
		routine.car_id = car.id;
		if (car.plan_time > total_time) {
			routine.start_time = car.plan_time;
		}
		else {
			routine.start_time = total_time;
		}
		temp = car.from;
		while (temp!=car.to)
		{
			next = topo.pathID[temp][car.to];
			routine.roads.push_back(topo.adjRoadID[temp][next]);
			temp = next;
		}
		
		temp_time = 0;
		for (auto j = 0; j < routine.roads.size(); ++j) {
			Road road = ins_->roads[routine.roads[j]];
			speed = min(road.speed, car.speed);
			temp_time += road.length / speed;
		}
		//std::cout << temp_time << std::endl;
		total_time = routine.start_time + temp_time;
		output_->routines.push_back(routine);
	}
	std::cout << "the cost time is" << total_time << std::endl;
}

void Solver::check_solution()
{
	int car_size = ins_->cars.size();
	int road_size = ins_->roads.size();
	vector<ID> *time_car;
	STATE *car_state;

	car_state = new STATE[car_size];
	time_car = new vector<ID>[total_time];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = output_->routines[i].start_time;
		time_car[start_time].push_back(output_->routines[i].car_id);
	}
	for (int i = 0; i <= MAX_TIME; i++) {
		for (int j = 0; j < road_size; ++j) {
			/* 对所有车道进行调整  */
			//if (topo.carsOnRoad->size() > 0)//先调度已经上路的车辆
			//{

			//}
		}
	}
}
    
Topo::Topo(Instance *ins) :ins_(ins), vexnum(ins->crosses.size()) {
	int rsize = ins->roads.size();
	sPathLen = new Length*[vexnum];
	adjRoadID = new ID*[vexnum];
	pathID = new ID*[vexnum];
	RoadTurn = new Turn*[rsize];
	/*carsWillOnRoad.resize(rsize);
	carsOnRoad.resize(rsize);*/
	for (int i = 0; i < vexnum; ++i) {
		sPathLen[i] = new Length[vexnum];
		adjRoadID[i] = new ID[vexnum];
		pathID[i] = new ID[vexnum];
	}
	for (int i = 0; i < vexnum; ++i) {
		for (int j = 0; j < vexnum; ++j) {
			adjRoadID[i][j] = INVALID_ID;
			sPathLen[i][j] = LENGTH_MAX;
			pathID[i][j] = INVALID_ID;
		}
	}
	for (int i = 0; i < rsize; ++i) {
		RoadTurn[i] = new Turn[rsize];

		ID from = ins->roads[i].from;
		ID to = ins->roads[i].to;
		adjRoadID[from][to] = i;
		pathID[from][to] = to;
		sPathLen[from][to] = ins->roads[i].length;
		if (ins->roads[i].is_duplex) {
			adjRoadID[to][from] = i;
			pathID[to][from] = from;
			sPathLen[to][from] = ins->roads[i].length;
		}
	}

	for (int i = 0; i < rsize; ++i) {
		for (int j = 0; j < rsize; ++j) {
			if (i == j) { 
				RoadTurn[i][j] = InvalidTurn;
				continue;
			}
			if (ins->roads[i].to == ins->roads[j].from) {
				RoadTurn[i][j] = getRoadTurn(ins->roads[i].to,ins->roads[i].id, ins->roads[j].id);
			}
			else if (ins->roads[i].is_duplex&&ins->roads[i].from == ins->roads[j].from) {
				RoadTurn[i][j] = getRoadTurn(ins->roads[i].from, ins->roads[i].id, ins->roads[j].id);
			}
			else if(ins->roads[i].to == ins->roads[j].to && ins->roads[j].is_duplex){
				RoadTurn[i][j] = getRoadTurn(ins->roads[i].to, ins->roads[i].id, ins->roads[j].id);
			}
			else if (ins->roads[i].is_duplex&&ins->roads[i].from == ins->roads[j].to &&ins->roads[j].is_duplex) {
				RoadTurn[i][j] = getRoadTurn(ins->roads[i].from, ins->roads[i].id, ins->roads[j].id);
			}
			else {
				RoadTurn[i][j] = InvalidTurn;
			}
		}
	}
	for (int i = 0; i < rsize; ++i) {
		for (int j = 0; j < rsize; ++j) {
			if (i != j) {
				cout << ins->changeToOriginalID(i, RoadMap) << "," << ins->changeToOriginalID(j, RoadMap)<<"  ";
				if(RoadTurn[i][j] == Left)cout << "Left" << endl;
				if (RoadTurn[i][j] == Right)cout << "Right" << endl;
				if (RoadTurn[i][j] == Front)cout << "Front" << endl;
				if (RoadTurn[i][j] == InvalidTurn)cout << "InvalidTurn" << endl;
			}
		}
	}
	Floyd();
	//printPath();
};

void Topo::Floyd()
{
	int csize = ins_->crosses.size();
	for (int i = 0; i < csize; i++) {
		for (int j = 0; j < csize; j++) {
			for (int k = 0; k < csize; k++) {
				Length select = (sPathLen[j][i] == LENGTH_MAX || sPathLen[i][k] == LENGTH_MAX)
					? LENGTH_MAX : (sPathLen[j][i] + sPathLen[i][k]);
				
				if (sPathLen[j][k] > select) {
					pathID[j][k] = pathID[j][i];
					sPathLen[j][k] = select;
				}
			}
		}
	}
}

void Topo::printPath()
{
	cout << "各个顶点对的最短路径：" << endl;
	int row = 0;
	int col = 0;
	int temp = 0;
	ID next;
	for (row = 0; row < this->vexnum; row++) {
		for (col = row + 1; col < this->vexnum; col++) {
			cout << "v" << to_string(row + 1) << "---" << "v" << to_string(col + 1) << " weight: "
				<< sPathLen[row][col] << " path: ";
			temp = row;
			//循环输出途径的每条路径。
			while (temp != col) {
				//cout << "-->" << "v" << to_string(temp + 1);
				next = pathID[temp][col];
				cout << ins_->changeToOriginalID(adjRoadID[temp][next],RoadMap) << ", ";
				temp = next;
			}
			cout << endl;
		}

		cout << endl;
	}
	
}

Turn Topo::getRoadTurn(ID cross_id, ID road1, ID road2)
{
	int r1, r2;
	Cross cross = ins_->crosses[cross_id];
	for (int i = 0; i < MAX_CROSS_ROAD_NUM; ++i) {
		if (cross.road[i] == road1)
			r1 = i;
		else if (cross.road[i] == road2)
			r2 = i;
	}
	return cross_roadTurn[r1][r2];
}

}
