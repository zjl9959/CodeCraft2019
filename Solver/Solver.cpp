#include "Solver.h"

#include <set>

using namespace std;

namespace codecraft2019 {
bool carL_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {//location大，车道号小的优先级高
	bool res;
	if (carL1->location > carL2->location) {//carL1->location > carL2->location
		return true;
	}
	else if (carL1->location == carL2->location){
		return carL1->channel_id < carL2->channel_id;
	}
	return false;
}
bool raux_sort(const RAux &r1, const RAux &r2) {
	return r1.cost_time < r2.cost_time;
}
bool car_time_id_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {//出发时间早，且id小的优先级高
	if (carL1->start_time < carL2->start_time)
		return true;
	else if(carL1->start_time == carL2->start_time)
		return carL1->car_id < carL2->car_id;
	return false;
}
void Solver::run() {
    init();
    binary_generate_solution();
	//init_solution_2();
	/*init_solution_once();
	if (check_solution(output_->routines,aux) == -1) {
		Log(FY_TEST) << " dead lock\n";
	}*/
    // 添加算法。。。
}

// 此函数用于测试IO的ID映射是否正确，与求解算法无关。
void Solver::testIO() {
	output_->routines.resize(ins_->raw_cars.size());
	for (int i = 0; i < ins_->raw_cars.size(); ++i) {
		output_->routines[i].car_id = ins_->raw_cars[i].id;
		output_->routines[i].start_time = ins_->raw_cars[i].plan_time;
		output_->routines[i].roads.push_back(ins_->raw_cars[i].from);
		output_->routines[i].roads.push_back(ins_->raw_cars[i].to);
	}
}

// 在运行算法之前需要做一些初始化的动作。
void Solver::init() {
    // 保存两点之间的最短路径到shortest_paths
    shortest_paths.reserve(topo.cross_size*topo.cross_size*topo.cross_size/3);
    shortest_paths.resize(topo.cross_size);
    Log(Log::ZJL) << "\t------shortest path--------" << endl;
    Log(Log::ZJL) << "cross size:" << topo.cross_size << endl;
    for (ID i = 0; i < topo.cross_size; ++i) {
        shortest_paths[i].resize(topo.cross_size);
        for (ID j = 0; j < topo.cross_size; ++j) {
            //Log(Log::ZJL) << "from " << i << " to " << j << ":";
            if (topo.pathID[i][j] != INVALID_ID) {
                ID temp = i;
                while (temp != j && temp != INVALID_ID) {
                    ID next = topo.pathID[temp][j];
                    shortest_paths[i][j].push_back(topo.adjRoadID[temp][next]);
                    temp = next;
                    //Log(Log::ZJL) << shortest_paths[i][j].back() << " ";
                }
            }
            //Log(Log::ZJL) << endl;
        }
    }
    // 其它数据的初始化。。。
}

void Solver::init_solution()
{
	int car_size = ins_->raw_cars.size();
	int cross_size = ins_->raw_crosses.size();
	ID temp, next;
	Time temp_time,latest_time=-1;
	Speed speed=0;
	//vector<Car> notPlanCar;
	for (auto i = 0; i < car_size; ++i) {
		Routine routine;
		RawCar *raw_car = &ins_->raw_cars[i];
		routine.car_id = raw_car->id;
		if (raw_car->plan_time > latest_time) latest_time = raw_car->plan_time;
		temp = raw_car->from;
		while (temp != raw_car->to)
		{
			next = topo.pathID[temp][raw_car->to];
			routine.roads.push_back(topo.adjRoadID[temp][next]);
			temp = next;
		}

		temp_time = 0;
		for (auto j = 0; j < routine.roads.size(); ++j) {
			RawRoad road = ins_->raw_roads[routine.roads[j]];
			speed = min(road.speed, raw_car->speed);
			temp_time += road.length / speed;
		}//计算没有拥堵时的耗时
		InterRoutine *interR = new InterRoutine(topo.cars[i],temp_time);
		output_->routines.push_back(routine);//使用最短路，此时routine的出发时间还未确定
		aux.car_same[raw_car->from][raw_car->to][raw_car->plan_time].push_back(interR);
	}

	
	/**接下来调整车辆的实际出发时间*/
	vector<Routine *> *time_car;
	time_car = new vector<Routine *>[latest_time+1];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = ins_->raw_cars[i].plan_time;
		time_car[start_time].push_back(&output_->routines[i]);
	}
	/*for (int i = 0; i <= latest_time; ++i) {
		if (time_car[i].size()>0){
			std::cout << "the plan time is" << i << " the carnum is"<<time_car[i].size()<<std::endl;
			int cnt = i;
			while (time_car[i].size() != 0)
			{
				int hasChoose = 0;
				for (int j = 0; j < time_car[i].size(); ++j) {
					if ((rand() % (time_car[i].size() - j)) < (150 - hasChoose)) {
						time_car[i][j]->start_time = cnt;
						hasChoose++;
					}
					else {
						time_car[i][j]->start_time = cnt + 1;
					}
				}

			}
		}
		
	}*/
	std::cout << "the latest time is" << latest_time << std::endl;
}

struct PairCmp {
    bool operator()(const pair<Time, ID> &lhs, const pair<Time, ID> &rhs) {
        if (lhs.first < rhs.first)
            return true;
        else if (lhs.first == rhs.first&&lhs.second < rhs.second)
            return true;
        else
            return false;
    }
};

// 构造初始解版本二
void Solver::binary_generate_solution() {
    Log(Log::ZJL) << "start binary search" << endl;
    int car_num_left = 1;
    int car_num_right = topo.car_size;
    const int total_car_num = ins_->raw_cars.size();
    output_->routines.clear();
    for (int i = 0; i < total_car_num; ++i) {   // 每辆车都走最短路径
        Routine routine;
        routine.car_id = ins_->raw_cars[i].id;
        routine.roads = shortest_paths[ins_->raw_cars[i].from][ins_->raw_cars[i].to];
        output_->routines.push_back(move(routine));
    }
    vector<Time> best_start_times;              // 最优的车辆出发时间
    best_start_times.reserve(total_car_num);
    Time best_schdeule_time = MAX_TIME;
    vector<pair<Time, ID>> run_time;       // 计算每辆车在路上最少花费的时间（车走最短路线）。
    run_time.reserve(total_car_num);
    for (auto car = ins_->raw_cars.begin(); car != ins_->raw_cars.end(); ++car) {
        run_time.push_back(make_pair(
            min_time_cost(car->id, car->from, car->to), car->id));
    }
    sort(run_time.begin(), run_time.end(), [](pair<Time, ID> &lhs, pair<Time, ID> &rhs) {
        return lhs.first < rhs.first; });  // 在路上耗时少的车辆先出发。
    int binsearch_turn = 1;
	int car_num_mid = 1;
    while (car_num_left <= car_num_right) {      // 二分调参大法
        Log(Log::ZJL) << "\t-----binary search turn " << binsearch_turn << "------" << endl;
		car_num_mid = car_num_left + ((car_num_right - car_num_left) / 2);
		Time current_time = changeTime(total_car_num, car_num_mid, run_time);
        Time cur_schedule_time = check_solution(output_->routines,aux);
		Log(Log::ZJL) << car_num_mid << endl;
        Log(Log::ZJL) << "current time:" << current_time << " schedule time:" << cur_schedule_time << endl;
        if (cur_schedule_time != -1) {                 // 解是合法的，说明可以提高car_num_mid
            car_num_left = car_num_mid + 1;
            //if (cur_schedule_time < best_schdeule_time) {  // 记录最优解，这里check_solution能否传回准确的调度时间？
            //    best_schdeule_time = cur_schedule_time;
            //    best_start_times = start_times;
            //}
        } else {
            car_num_right = car_num_mid - 1;
        }
    }
	{
		car_num_mid -= 2;
		if (check_solution(output_->routines, aux) == -1) {
			car_num_mid -= 1;
		}
		Time current_time = changeTime(total_car_num, car_num_mid, run_time);
		//Time cur_schedule_time = check_solution(output_->routines,aux);
		Log(Log::ZJL) << car_num_mid << endl;
		//Log(Log::ZJL) << "current time:" << current_time << " schedule time:" << cur_schedule_time << endl;
		

	}
	generate_futher_solution();

}

void Solver::generate_futher_solution()
{
	vector<vector<Routine>> time_routine;
	time_routine.resize(20000);
	for (int i = 0; i < output_->routines.size(); ++i) {
		Time start_time = output_->routines[i].start_time;
		time_routine[start_time].push_back(output_->routines[i]);
	}
	Time time = check_solution(time_routine[1], aux);
	int temp_t;
	for (temp_t = 2; temp_t < time_routine.size(); ++temp_t) {
		if (time_routine[temp_t].size() > 0) {
			break;
		}
	}
	Time gap = temp_t - time;
	for (int i = 0; i < output_->routines.size(); ++i) {
		Time start_time = output_->routines[i].start_time;
		if (start_time > 1)
			output_->routines[i].start_time -= gap;
	}
	Time end_time = check_solution(output_->routines, aux);
	Log(FY_TEST) << end_time << endl;
	/*time_routine.clear();
	time_routine.resize(20000);

	for (int i = 0; i < output_->routines.size(); ++i) {
		Time start_time = output_->routines[i].start_time;
		time_routine[start_time].push_back(output_->routines[i]);
	}
	for (int t = 0; t < time_routine.size(); ++t) {
		if (time_routine[t].size() > 0) {
			for (int i = 0; i < time_routine[t].size(); ++i) {
				temp_routines.push_back(time_routine[t][i]);
			}
			Time time = check_solution(temp_routines, aux);
			for (int t1 = t + 1; t1 < time_routine.size(); ++t1) {
				if (time_routine[t1].size() > 0) {

				}
			}
			Log(FY_TEST) << t << "  :"<<time <<" size:"<<time_routine[t].size()<< endl;
		}

	}*/
}

Time Solver::changeTime(int total_car_num,int car_num_mid, vector<pair<Time, ID>> &run_time)
{
	vector<Time> start_times;
	start_times.resize(total_car_num);
    multiset<pair<Time, ID>, PairCmp> cars_run;
	int start_car_num = 0;
	Time current_time = 0;
	vector<pair<Time, ID>> cars_run_time(run_time);
	while (start_car_num < total_car_num) { // 按照不同的参数配置为每辆车设置出发时间。
		for (int i = 0; i < cars_run_time.size() && cars_run.size() < car_num_mid; ++i) {
			ID car_id = cars_run_time[i].second;
			if (cars_run_time[i].first == MAX_TIME)
				break;
			if (ins_->raw_cars[car_id].plan_time <= current_time) {
                cars_run.insert(make_pair(cars_run_time.back().first + current_time, car_id));
				start_times[car_id] = current_time;
				++start_car_num;
				cars_run_time[i].first = MAX_TIME;
				// Log(Log::ZJL) << "car " << car_id << " start at time " << current_time << endl;
			}
		}
		sort(cars_run_time.begin(), cars_run_time.end(), [](pair<Time, ID> &lhs, pair<Time, ID> &rhs) {
            return lhs.first < rhs.first; });
		while (!cars_run.empty()) {           // 弹出到达目的地的车辆
			if ((*cars_run.begin()).first <= current_time) {
                cars_run.erase(cars_run.begin());
			}
			else {
				break;
			}
		}
		current_time++;
	}
	for (int i = 0; i < start_times.size(); ++i) {
		output_->routines[i].start_time = start_times[i];
	}
	return current_time;
}


int Solver::check_solution(const vector<Routine> &routines,Aux &aux)
{
	vector<vector<ID>> time_car;
	time_car.resize(100000);
    timeslice.clear();
	for (int i = 0; i < routines.size(); ++i) {
		Time start_time = routines[i].start_time;
		time_car[start_time].push_back(i);
	}
	inDst_num = 0;
	cars_totalTime = 0;
	
	for ( t = 0; t <= MAX_TIME; t++) {
		if (t == 5) {
			int j;
			j = t* 1;
		}

		for (int r = 0; r <double_road_size; ++r) {
			/* 对所有车道进行调整  */
			Road *road = topo.roads[r];
			driveAllCarJustOnRoadToEndState(road);
		}

		/*调度等待运行的车辆*/
		for (int c = 0; c < cross_size; ++c) {//先将所有等待过路口的记录下来
			Cross *cross = topo.crosses[c];

			for (int k = 0; k < cross->road.size(); ++k) {//确定哪些车辆需要调度
				Road *road = cross->road[k];
				//获取当前道路上的车辆，当前需要做的是确定道路内等待行驶的车辆的状态
				//可以出路口的车辆有哪些
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {//车道
					if (road->channel_carL[ch].size() > 0) {
						CarLocationOnRoad *carL = road->channel_carL[ch][0];//某一个车道内的第一辆车
						/*Log(FY_TEST) << "car_id: " <<ins_->changeToOriginalID( carL->car_id ,CarMap)<< "  location: " << carL->location
							<< "  road_id :" <<ins_->changeToOriginalID( road->raw_road->id ,RoadMap)<< endl;*/
						if (carL->state == STATE_waitRun) {//车道内第一辆车为等待状态，则该车可能需要出路口
							recordProbOutCross(carL, cross, road,routines);
						
							for (int cL = 1; cL < road->channel_carL[ch].size(); cL++) {//将该车道内所有可能其它出路口的车保存
								CarLocationOnRoad *next_carL = road->channel_carL[ch][cL];
								Speed speed = min(road->raw_road->speed, ins_->raw_cars[next_carL->car_id].speed);
								if (speed + next_carL->location > road->raw_road->length && next_carL->state == STATE_waitRun) {
									recordProbOutCross(next_carL, cross, road, routines);
								}
								else {//车道前的车速度无法出路口，那么后面的车更加不会
									break;
								}
							}

						}
					}
					
				}
				//对可能可以出路口的车辆进行优先级排序
				sort(road->waitOutCarL.begin(), road->waitOutCarL.end(), carL_sort);
			}
		}

		while (true) {
			bool canSchedule = false;
			for (int c = 0; c < cross_size; ++c) {//按照ID升序调度各个路口
				Cross *cross = topo.crosses[c];
				/*按照道路ID升序对车辆进行调度*/
				while (true) {
					bool isCarRun = false;
					for (int k = 0; k < cross->road.size(); ++k) {
						Road *road = cross->road[k];//如何判断冲突
						bool conflict = false;
						if (t == 3 && c == 37) {
							int j;
							j = t * 1;
						}

						int old_size = road->waitOutCarL.size();
						for (int r = 0; r < road->waitOutCarL.size(); ++r) {//按照优先级顺序依次遍历
							CarLocationOnRoad *carL = road->waitOutCarL[r];
							if (r != 0) {
								Log(FY_TEST) << "error\n";
								exit(2);
							}
							if (carL->state == STATE_terminated) {
								Log(FY_TEST) << "error\n";

							}
							Road *next_road = getNextRoad(carL, cross, routines);

							if (carL->turn != Front ) {//直行的优先级要高于其它,不会有冲突
								for (int k1 = 0; k1 < cross->road.size(); ++k1) {
									if (k1 != k) {
										Road *other_road = cross->road[k1];
										if (other_road->waitOutCarL.size() > 0) {
											CarLocationOnRoad *first_carL = other_road->waitOutCarL[0];//获取第一优先级车辆
											Road *other_next_road = getNextRoad(first_carL, cross, routines);
											if (other_next_road == NULL) {//其它车道上到达终点的车辆，要考虑直行冲突
												if (topo.RoadTurn[other_road->raw_road->id][next_road->raw_road->id] == Front) {
													conflict = true;
													break;
												}
											}
											else if ((other_next_road == next_road && first_carL->turn < carL->turn)) {
												conflict = true;
												break;
											}
										}
									}
								}
								if (conflict)//有冲突此次调度结束
									break;
							}
							int temp_ch = carL->channel_id; // 原道路所在的车道
							if (moveToNextRoad(road, next_road, carL) ||
								carL->state == STATE_terminated) {//说明在该时刻已经不可能出路口
								road->waitOutCarL.erase(road->waitOutCarL.begin());
								driveCarOnChannelToEndState(road, temp_ch);
								isCarRun = true;
								canSchedule = true;
								r = -1;
							}
							else {//第一优先级不能出路口，那么后面的车都不能出
								break;
							}
						}
					}
					if (!isCarRun)
						break;
				}
			}
			if (!canSchedule)
				break;
		}

		for (int c = 0; c < cross_size; ++c) {
			Cross *cross = topo.crosses[c];
			for (int k = 0; k < cross->road.size(); ++k) {
				Road *road = cross->road[k];
				if (road->waitOutCarL.size() > 0) {
					clearRoadVector(routines.size());
					return -1;
				}
			}
		}
		/*for (int c = 0; c < cross_size ; ++c) {
			Cross *cross = topo.crosses[c];
			for (int k = 0; k < cross->road.size(); ++k) {
				Road *road = cross->road[k];
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
					for (int cl = 0; cl < road->channel_carL[ch].size(); ++cl) {
						CarLocationOnRoad *carL = road->channel_carL[ch][cl];
						if (carL->state == STATE_waitRun) {
							return -1;
						}
					}
				}
			}
		}*/
		if (inDst_num == routines.size()) {
			/*cout << " the cost time of scheduler is " << t << endl;
			cout << " the total time is" << cars_totalTime<<endl;*/
			clearRoadVector(routines.size());
			return t;
		}
		for (int j = 0; j < time_car[t].size(); j++) {//将所有该时刻要出发的车辆记录下来
			ID first_road = routines[time_car[t][j]].roads[0];
			ID car_id = routines[time_car[t][j]].car_id;
			CarLocationOnRoad *new_carL = new CarLocationOnRoad;
			new_carL->car_id = car_id;
			new_carL->indexInRoutine = time_car[t][j];
			new_carL->index = 0;
			new_carL->state = STATE_start;
			new_carL->start_time = t;
			if (ins_->raw_roads[first_road].from == ins_->raw_cars[car_id].from) {
				topo.roads[first_road]->willOnRoad.push_back(new_carL);
			}
			else {
				topo.roads[first_road +road_size]->willOnRoad.push_back(new_carL);
			}
		}
		driveCarInGarage();

#if FY_TEST == 0
		//Log(FY_TEST) << "\n\n------------------- Time  " << t << "-------------------\n" << endl;

		vector<vector<CarInfo>> road_carInfos;
		road_carInfos.resize(road_size);
		for (int i = 0; i < road_size ; ++i) {
			Road *road = topo.roads[i];
			vector<CarInfo> carInfos; //可视化信息
			for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
				for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
					CarLocationOnRoad *carL = road->channel_carL[ch][cL];
					CarInfo carinfo(carL->car_id,carL->location,road->raw_road->channel -1 - carL->channel_id,carL->state);
					carInfos.push_back(carinfo);
				}
			}
			if (road->raw_road->is_duplex) {
				Road *road = topo.roads[i + road_size];
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
					for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
						CarLocationOnRoad *carL = road->channel_carL[ch][cL];
						CarInfo carinfo(carL->car_id, carL->location, road->raw_road->channel +carL->channel_id, carL->state);
						carInfos.push_back(carinfo);
					}
				}
			}
			road_carInfos[i] = carInfos;
		}
		if(t == 1)
			timeslice.push_back(road_carInfos);
		for (int i = 0; i < road_size * 2; ++i) {
			
			Road *road = topo.roads[i];
			if (road != NULL) {
				
				for (int c = 0; c < road->channel_carL.size(); ++c) {
					if (road->channel_carL[c].size() > 0) {
						for (int cL = road->channel_carL[c].size() - 1; cL >= 0; --cL) {
							CarLocationOnRoad *carL = road->channel_carL[c][cL];
							if (routines[carL->indexInRoutine].roads[carL->index] != road->raw_road->id) {
								cout << "road  error" << endl;

							}
							if (carL->channel_id != c) {
								cout << "error" << endl;
							}
						}
						/*Log(FY_TEST) << road->raw_road->id << ": " << ins_->changeToOriginalID(road->from_id, CrossMap) << "\t"
							<< ins_->changeToOriginalID(road->to_id, CrossMap) << endl;

						for (int len = 1; len <= road->raw_road->length; len++) {
							Log(FY_TEST) << "---" << len << "--- ";
						}

						Log(FY_TEST) << endl;


						int loc = 0;
						for (int cL = road->channel_carL[c].size() - 1; cL >= 0; --cL) {
							CarLocationOnRoad *carL = road->channel_carL[c][cL];
							for (int t = loc; t < carL->location - 1; t++) {
								cout << "       |";
							}
							Log(FY_TEST) << ins_->changeToOriginalID(carL->car_id, CarMap) << "  |";
							loc = carL->location;
						}
						Log(FY_TEST) << endl;*/
					}

				}

				for (int c = 0; c < road->channel_carL.size(); ++c) {
					if (road->channel_carL[c].size() > 0) {
						for (int cL = road->channel_carL[c].size() - 1; cL >= 0; --cL) {
							CarLocationOnRoad *carL = road->channel_carL[c][cL];
							for (int cL1 = road->channel_carL[c].size() - 1; cL1 >= 0; --cL1) {
								if (cL1 == cL) continue;
								CarLocationOnRoad *carL1 = road->channel_carL[c][cL1];
								if (carL->location == carL1->location)
								{
									cout << "In road " << road->raw_road->id << "location error";
								}
							}
						}
					}

				}
			}


		}
		
#endif // FY_TEST
	}
}

void Solver::driveCarOnChannelToEndState(Road *road, int ch) {//调度处于等待状态，且仍然在车道里的车辆
	for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
		CarLocationOnRoad *carL = road->channel_carL[ch][cL];//车道内的车辆id及位置
		Speed speed = min(ins_->raw_cars[carL->car_id].speed, road->raw_road->speed);

		if (carL->state == STATE_terminated ) { continue; }

		if (cL == 0) {//说明是车道内的第一辆车
			if (carL->location + speed > road->raw_road->length) {//会出路口
				carL->state = STATE_waitRun;
			}
			else {
				carL->location = carL->location + speed;
				carL->state = STATE_terminated;
			}
		}
		else {
			CarLocationOnRoad *prev_carL = road->channel_carL[ch][cL - 1];
			if (prev_carL->location - carL->location > speed) {//前车距离大于该车1个时间单位内可行驶的最大距离，视作无阻挡
				carL->location += speed;
				carL->state = STATE_terminated;
			}
			else {
				if (prev_carL->state == STATE_terminated) {
					if (carL->location + speed > road->raw_road->length) {//前车确定了为终止状态之后，后面要出路口的车将确定为终止状态，并从vector中删除
						for (vector<CarLocationOnRoad*>::iterator iter = road->waitOutCarL.begin();
							iter != road->waitOutCarL.end(); ++iter) {
							CarLocationOnRoad *temp_carL = *iter;
							if (temp_carL == carL) {
								road->waitOutCarL.erase(iter);
								break;
							}
						}
					}
					speed = min(prev_carL->location - carL->location - 1, speed);
					carL->location += speed;
					carL->state = STATE_terminated;
					
				}
				else {
					carL->state = STATE_waitRun;
				}
			}
		}
	}
}
void Solver::driveAllCarJustOnRoadToEndState(Road *road)
{
	if (road == NULL) return;
	for (int c = 0; c < road->channel_carL.size(); ++c) {//获取车道
		for (int cL = 0; cL < road->channel_carL[c].size(); ++cL) {
			CarLocationOnRoad *carL = road->channel_carL[c][cL];//车道内的车辆id及位置
			Speed speed = min(ins_->raw_cars[carL->car_id].speed, road->raw_road->speed);
			
			if (cL == 0) {//说明是车道内的第一辆车
				if (carL->location + speed > road->raw_road->length) {//会出路口
					carL->state = STATE_waitRun;
				}
				else {
					carL->location = carL->location + speed;
					carL->state = STATE_terminated;
				}
			}
			else {
				CarLocationOnRoad *prev_carL = road->channel_carL[c][cL - 1];
				if (prev_carL->location - carL->location > speed) {//前车距离大于该车1个时间单位内可行驶的最大距离，视作无阻挡
					carL->location += speed;
					carL->state = STATE_terminated;
				}
				else 
				{
					if (prev_carL->state == STATE_terminated) {
						carL->location = prev_carL->location  - 1;
						carL->state = STATE_terminated;
					}
					else {
						carL->state = STATE_waitRun;
					}
				}
			}
		}
	}
}
void Solver::driveCarInGarage()
{
	int rsize = road_size * 2;
	for (int i = 0; i < rsize; ++i) {
		Road *road = topo.roads[i];
		if (road) {//假如有这样的路径
			sort(road->willOnRoad.begin(), road->willOnRoad.end(), car_time_id_sort);
			for (int j = 0; j < road->willOnRoad.size(); ++j) {//按照id顺序驶入道路
				CarLocationOnRoad * carLWillOnRoad = road->willOnRoad[j]; 
				Speed speed = min(road->raw_road->speed, ins_->raw_cars[carLWillOnRoad->car_id].speed);
				bool canDriveIn = false;
				for (int c = 0; c < road->channel_carL.size(); ++c) {//遍历道路上所有的车道
					if (road->channel_carL[c].size() == 0) {
						carLWillOnRoad->channel_id = c;
						carLWillOnRoad->location = speed;
						road->channel_carL[c].push_back(carLWillOnRoad);
						canDriveIn = true;
						break;
					}
					else {
						CarLocationOnRoad *carL = road->channel_carL[c].back();
						if (carL->location > 1) {
							speed = min(carL->location - 1, speed);
							carLWillOnRoad->channel_id = c;
							carLWillOnRoad->location = speed;
							road->channel_carL[c].push_back(carLWillOnRoad);
							canDriveIn = true;
							break;
						}
					}
				}
				if (canDriveIn) {
					road->willOnRoad.erase(road->willOnRoad.begin());
					j = -1;
				}
				else {//后面的车也无法驶入，直接break
					break;
				}

			}
		}
	}

}
/*
* carL: 可能要出当前路口的carL
* cross: 当前路口
* road: 当前道路
*/
void Solver::recordProbOutCross(CarLocationOnRoad * carL, Cross * cross, Road * road,const vector<Routine> &routines)
{
	Turn turn =Front;
	if (carL->index + 1 < routines[carL->indexInRoutine].roads.size()) {
		ID next_road_id = routines[carL->indexInRoutine].roads[carL->index + 1];//这里之后可能会做修改
		turn = topo.RoadTurn[road->raw_road->id][next_road_id];
	}
	
	carL->turn = turn;
	if (turn == InvalidTurn) {
		Log(FY_TEST) <<" turn error";
		exit(1);
	}
	road->waitOutCarL.push_back(carL);
}

void Solver::clearRoadVector(int size)
{
	for (int c = 0; c < cross_size; ++c) {
		Cross *cross = topo.crosses[c];
		for (int k = 0; k < cross->road.size(); ++k) {
			Road *road = cross->road[k];
			for (int ch = 0; ch < road->channel_carL.size(); ++ch) {

				road->channel_carL[ch].clear();
			}
			road->waitOutCarL.clear();
			road->willOnRoad.clear();
		}
	}
	/*if (carL_inDst.size() != size) {
		Log(FY_TEST) << "in dst size error" << endl;
		exit(4);
	}*/
	for (int i = 0; i < carL_inDst.size(); ++i) {
		delete carL_inDst[i];
	}
	carL_inDst.clear();
}

Road* Solver::getNextRoad(CarLocationOnRoad * carL, Cross * cross, const vector<Routine> &routines)
{
	Road *next_road = NULL;
	if (carL->index + 1 < routines[carL->indexInRoutine].roads.size()) {
		ID next_road_id = routines[carL->indexInRoutine].roads[carL->index + 1];//这里之后可能会做修改
		Road *road1 = topo.roads[next_road_id],*road2 = topo.roads[next_road_id + road_size];
		if (road1 && cross->raw_cross->id == road1->from_id) {
			next_road = road1;
		}
		if (road2 && cross->raw_cross->id == road2->from_id) {
			next_road = road2;
		}
		if (next_road == NULL) {
			Log(FY_TEST) << "next road is NULL\n";
			exit(3);
		}
	}
	return next_road;
}

bool Solver::moveToNextRoad(Road * road, Road * next_road, CarLocationOnRoad * carL)
{
	Length S1 = road->raw_road->length - carL->location;//当前道路的可行驶距离
	if (next_road == NULL) {
		Speed speed = min(road->raw_road->speed, ins_->raw_cars[carL->car_id].speed);
		if (speed > S1) {
			carL->state = STATE_in_dst;
			road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());//第一个元素是最靠近路口的车辆
			inDst_num++;
			carL_inDst.push_back(carL);
			//cout << t << endl;
			/*cars_totalTime += t - output_->routines[carL->car_id].start_time;*/
		}
		else {
			cout << "out error";
			exit(1);
		}
		return true;
	}
	else {
		Speed V2 = min(next_road->raw_road->speed, ins_->raw_cars[carL->car_id].speed);//下一条道路可行驶的最大速度
		if (S1 >= V2) {//该车不能通过路口
			
			carL->location = road->raw_road->length;
			carL->state = STATE_terminated;
			return false;
		}
		for (int c = 0; c < next_road->channel_carL.size(); ++c) {//遍历下一条路的所有车道
			if (next_road->channel_carL[c].size() == 0) {
				road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());
				carL->location = V2 - S1;
				carL->channel_id = c;
				carL->state = STATE_terminated;
				carL->index += 1;
				next_road->channel_carL[carL->channel_id].push_back(carL);
				return true;
			}
			else {
				CarLocationOnRoad *last_carL = next_road->channel_carL[c].back();//某一车道的最后一辆车
				if (last_carL->state == STATE_terminated ) {//车道内的车进入了终止状态
					if (last_carL->location > 1) {//有空位
						road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());//第一个元素是最靠近路口的车辆
						carL->location = min(last_carL->location - 1, V2 - S1);
						carL->channel_id = c;
						carL->state = STATE_terminated;
						carL->index += 1;
						next_road->channel_carL[carL->channel_id].push_back(carL);
						return true;
					}
					else if (last_carL->location == 1 && c == (next_road->channel_carL.size() - 1)) {
						carL->location = road->raw_road->length;
						carL->state = STATE_terminated;
						return false;
					}
					
				}
				else if (last_carL->state == STATE_waitRun) {//最后一辆车为等待状态
					if (last_carL->location <= V2 - S1)//有阻挡
						return false;
					else {
						road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());
						carL->location = V2 - S1;
						carL->channel_id = c;
						carL->state = STATE_terminated;
						carL->index += 1;
						next_road->channel_carL[carL->channel_id].push_back(carL);
						return true;
					}
				}
			}
			
		}
	}
	
	return false;
}

// 计算一辆车从一个路口到另一个路口的最短耗时（假设不会堵车）
Time Solver::min_time_cost(const ID car, const ID from, const ID to) const {
    Time time_cost = 0;
    const Speed car_speed = ins_->raw_cars[car].speed;
    Length run_more_length = 0;             // 在下一条道路上能够走的距离
    for (auto road = shortest_paths[from][to].begin();
        road != shortest_paths[from][to].end(); ++road) {
        Length this_road_length = ins_->raw_roads[*road].length - run_more_length;
        if (this_road_length <= 0) {        // 跑得贼鸡儿快，都过了下一个路口了。
            run_more_length = 0;            // 如果道路长度大于道路最高限速，这种情况就不会发生。
            continue;
        }
        Speed this_road_speed = ins_->raw_roads[*road].speed;
        Time this_road_cost_time = this_road_length / min(this_road_speed, car_speed);
        time_cost += this_road_cost_time;
        Length this_road_left_length =  // this_road_cost_time时间单位后，到达当前路口所剩余的距离。
            this_road_length - this_road_cost_time * min(this_road_speed, car_speed);
        if (!this_road_left_length)continue;
        if ((road + 1) != shortest_paths[from][to].end()) {
            Speed next_road_speed = ins_->raw_roads[*(road + 1)].speed;
            run_more_length = min(next_road_speed, car_speed) > this_road_left_length ?
                min(next_road_speed, car_speed) - this_road_left_length : 0;
        }
        ++time_cost;    // 这一秒走完剩余的路，无论是不是最后一条路。
    }
    return time_cost;
}

void Solver::read_from_file()
{

}


void Solver::init_solution_once() {
	int car_size = ins_->raw_cars.size();
	ID temp, next;
	Time  temp_time;
	Speed speed;
	vector<Car> notPlanCar;
	for (auto i = 0; i < car_size; ++i) {
		Routine routine;
		RawCar car = ins_->raw_cars[i];
		routine.car_id = car.id;
		routine.start_time = car.plan_time;
		temp = car.from;
		while (temp != car.to)
		{
			next = topo.pathID[temp][car.to];
			routine.roads.push_back(topo.adjRoadID[temp][next]);
			temp = next;
		}

		temp_time = 0;
		for (auto j = 0; j < routine.roads.size(); ++j) {
			RawRoad road = ins_->raw_roads[routine.roads[j]];
			speed = min(road.speed, car.speed);
			temp_time += road.length / speed;
		}
		total_time += temp_time;
		output_->routines.push_back(routine);
	}
	/*for (auto i = 0; i < 5; ++i) {
		Routine routine;
		RawCar car = ins_->raw_cars[i];
		routine.car_id = car.id;
		routine.start_time = car.plan_time;
		routine.roads.push_back(26);
		routine.roads.push_back(31);
		routine.roads.push_back(36);
		routine.roads.push_back(30);
		routine.roads.push_back(24);

		output_->routines.push_back(routine);
	}
	for (auto i = 5; i < car_size; ++i) {
		Routine routine;
		RawCar car = ins_->raw_cars[i];
		routine.car_id = car.id;
		routine.start_time = car.plan_time;
		routine.roads.push_back(35);
		routine.roads.push_back(30);
		routine.roads.push_back(25);
		routine.roads.push_back(31);
		routine.roads.push_back(37);

		output_->routines.push_back(routine);
	}*/
	total_time = 100;
	std::cout << "the cost time is " << total_time << std::endl;
}
void Solver::init_solution_2()
{
	rauxs.resize(car_size);
	for (int i = 0; i < car_size; ++i) {   // 每辆车都走最短路径
		Routine routine;
		RAux raux;
		routine.car_id = ins_->raw_cars[i].id;
		routine.roads = shortest_paths[ins_->raw_cars[i].from][ins_->raw_cars[i].to];
		Speed car_speed = ins_->raw_cars[i].speed;
		Time cost_time =0;
		for (int i = 0; i < routine.roads.size(); ++i) {
			RawRoad *raw_road = &ins_->raw_roads[routine.roads[i]];
			Speed speed = min(car_speed, raw_road->speed);
			cost_time += raw_road->length / speed;
		}
		raux.cost_time = cost_time;
		raux.raw_car = &ins_->raw_cars[i];
		raux.car_id = routine.car_id;
		rauxs.push_back(move(raux));
		output_->routines.push_back(move(routine));
	}
	sort(rauxs.begin(), rauxs.end(), raux_sort);//按照消耗时间排序
	int car_numInTime = 20;
	vector<Routine> end_routines;
	for (int t = 0; t <= MAX_TIME ; ++t) {
		if (rauxs.size() == 0) break;
		
		int  cnt = 0;
		for (vector<RAux>::iterator iter = rauxs.begin();
			iter <= rauxs.end(); ++iter) {
			if (cnt > car_numInTime)break;
			if ((*iter).raw_car->plan_time <= t) {//说明可以在该时刻出发
				cnt++;
				output_->routines[(*iter).car_id].start_time = t;
				end_routines.push_back(move(output_->routines[(*iter).car_id]));
				temp_routines.push_back(move(output_->routines[(*iter).car_id]));
			}
		}
		Time cost_time = check_solution(temp_routines, aux);
	}
}
}