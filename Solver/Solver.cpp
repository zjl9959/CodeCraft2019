#include "Solver.h"

using namespace std;

namespace codecraft2019 {
bool carL_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {//location大，车道号小的优先级高
	bool res;
	if (carL1->location > carL2->location) {//carL1->location >= carL2->location
		return true;
	}
	else if (carL1->location == carL2->location){
		return carL1->channel_id < carL2->channel_id;
	}
	return false;
}

bool car_id_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {
	return carL1->car_id < carL2->car_id;
}
void Solver::run() {
    init();
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
    for (ID i = 0; i < topo.cross_size; ++i) {
        shortest_paths[i].resize(topo.cross_size);
        for (ID j = 0; j < topo.cross_size; ++j) {
            if (topo.pathID[i][j] != INVALID_ID) {
                ID temp = i;
                while (temp != j) {
                    ID next = topo.pathID[temp][j];
                    shortest_paths[i][j].push_back(topo.adjRoadID[temp][next]);
                    temp = next;
                }
            }
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


// 构造初始解版本二
void Solver::binary_generate_solution() {
    int car_num_left = 1;
    int car_num_right = topo.car_size;
    const int total_car_num = ins_->raw_cars.size();
    vector<Time> best_start_times;              // 最优的车辆出发时间
    best_start_times.reserve(total_car_num);
    Time best_schdeule_time = MAX_TIME;
    vector<pair<Time, ID>> cars_run_time;         // 计算每辆车在路上最少花费的时间（车走最短路线）。
    cars_run_time.reserve(total_car_num);
    for (auto car = ins_->raw_cars.begin(); car != ins_->raw_cars.end(); ++car) {
        cars_run_time.push_back(make_pair(
            min_time_cost(car->id, car->from, car->to), car->id));
    }
    //auto cmp_less = [](pair<Time, ID> &lhs, pair<Time, ID> &rhs) {return lhs.first < rhs.first; };
    auto cmp_greater = [](pair<Time, ID> &lhs, pair<Time, ID> &rhs) { return lhs.first > rhs.first; };
    sort(cars_run_time.begin(), cars_run_time.end(), cmp_greater);  // 在路上耗时少的车辆先出发。
    while (car_num_left < car_num_right) {      // 二分调参大法
        int car_num_mid = car_num_left + ((car_num_right - car_num_left) / 2);
        vector<Time> start_times;
        start_times.reserve(total_car_num);
        priority_queue<pair<Time, ID>, vector<pair<Time,ID>>, decltype(cmp_greater)> pqueue(cmp_greater);
        int start_car_num = 0;
        Time current_time = 0;
        while (start_car_num < total_car_num) { // 按照不同的参数配置为每辆车设置出发时间。
            for (auto it = cars_run_time.rbegin(); it != cars_run_time.rend() && pqueue.size() < car_num_mid; ++it) {
                ID car_id = cars_run_time.back().second;
                if (ins_->raw_cars[car_id].plan_time <= current_time) {
                    pqueue.push(make_pair(cars_run_time.back().first + current_time, car_id));
                    start_times[car_id] = current_time;
                    ++start_car_num;
                    cars_run_time.pop_back();
                }
            }
            while (!pqueue.empty()) {           // 弹出到达目的地的车辆
                if (pqueue.top().first <= current_time) {
                    pqueue.pop();
                } else {
                    break;
                }
            }
            current_time++;
        }
        // [TODO]这里得把解得参数给传过去
        if (check_solution()) {                 // 解是合法的，说明可以提高car_num_mid
            car_num_left = car_num_mid + 1;
            if (current_time < best_schdeule_time) {  // 记录最优解，这里check_solution能否传回准确的调度时间？
                best_schdeule_time = current_time;
                best_start_times.swap(start_times);
            }
        } else {
            car_num_right = car_num_mid - 1;
        }
    }
    // [TODO] 最后将初始解转换成需要的形式。
}

int Solver::check_solution()
{
	
	vector<ID> *time_car;
	STATE *car_state;

	car_state = new STATE[car_size];
	time_car = new vector<ID>[total_time+3];
	for (int i = 0; i < car_size; ++i) {
		Time start_time = output_->routines[i].start_time;
		time_car[start_time].push_back(i);
	}
	inDst_num = 0;
	for (int j = 0; j < car_size; ++j) {
		Log(FY_TEST) << "car_id:" << ins_->changeToOriginalID(j, CarMap)<<"  ";
		for (int i = 0; i < output_->routines[j].roads.size(); ++i) {
			Log(FY_TEST) << ins_->changeToOriginalID(output_->routines[j].roads[i], RoadMap)<< "  ";
		}
		Log(FY_TEST) << endl;
	}
	
	for (int i = 0; i <= MAX_TIME; i++) {
		if (i == 12) {
			int j;
			j = i* 1;
		}

		for (int r = 0; r < road_size; ++r) {
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
							recordProbOutCross(carL, cross, road);

							for (int cL = 1; cL < road->channel_carL[ch].size(); cL++) {//将该车道内所有可能其它出路口的车保存
								CarLocationOnRoad *next_carL = road->channel_carL[ch][cL];
								Speed speed = min(road->raw_road->speed, ins_->raw_cars[next_carL->car_id].speed);
								if (speed + next_carL->location > road->raw_road->length) {
									recordProbOutCross(next_carL, cross, road);
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
						int old_size = road->waitOutCarL.size();
						for (int r = 0; r < road->waitOutCarL.size(); ++r) {//按照优先级顺序依次遍历
							CarLocationOnRoad *carL = road->waitOutCarL[r];
							Road *next_road = getNextRoad(carL, cross);

							if (carL->turn != Front ) {//直行的优先级要高于其它,不会有冲突
								for (int k1 = 0; k1 < cross->road.size(); ++k1) {
									if (k1 != k) {
										Road *other_road = cross->road[k1];
										if (other_road->waitOutCarL.size() > 0) {
											CarLocationOnRoad *first_carL = other_road->waitOutCarL[0];//获取第一优先级车辆
											Road *other_next_road = getNextRoad(first_carL, cross);
											if (other_next_road == next_road && first_carL->turn < carL->turn) {
												conflict = true;
												break;
											}
										}
									}
								}
								if (conflict)//有冲突此次调度结束
									break;
							}
							if (moveToNextRoad(road, next_road, carL) ||
								carL->state == STATE_terminated) {//说明在该时刻已经不可能出路口
								road->waitOutCarL.erase(road->waitOutCarL.begin());
								driveCarOnChannelToEndState(road, carL->channel_id);
								isCarRun = true;
								canSchedule = true;
								r = -1;
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

		bool isValid = true;
		for (int c = 0; c < cross_size && isValid; ++c) {
			Cross *cross = topo.crosses[c];
			for (int k = 0; k < cross->road.size(); ++k) {
				Road *road = cross->road[k];
				if (road->waitOutCarL.size() > 0) {
					isValid = false;
					break;
				}
			}
		}
		if (!isValid) {
			return -1;
		}

		driveCarInGarage();
		if (inDst_num == car_size) {
			cout << " the cost time of scheduler is " << i << endl;
			return i;
		}
		for (int j = 0; j < time_car[i].size(); j++) {//将所有该时刻要出发的车辆记录下来
			ID first_road = output_->routines[time_car[i][j]].roads[0];
			ID car_id = output_->routines[time_car[i][j]].car_id;
			CarLocationOnRoad *new_carL = new CarLocationOnRoad;
			new_carL->car_id = car_id;
			new_carL->index = 0;
			new_carL->state = STATE_start;
			if (ins_->raw_roads[first_road].from == ins_->raw_cars[car_id].from) {
				topo.roads[first_road]->willOnRoad.push_back(new_carL);
			}
			else {
				topo.roads[first_road +road_size]->willOnRoad.push_back(new_carL);
			}
		}

#ifdef FY_TEST
		cout << "------------------- Time  " << i << "-------------------" << endl;
		for (int i = 0; i < road_size * 2; ++i) {
			Road *road = topo.roads[i];
			for (int c = 0; c < road->channel_carL.size(); ++c) {
				for (int cL = 0; cL < road->channel_carL[c].size(); ++cL) {
					CarLocationOnRoad *carL = road->channel_carL[c][cL];
					Log(FY_TEST) << "car_id: " << ins_->changeToOriginalID(carL->car_id, CarMap) << "  location: " << carL->location
						<< "  road_id :" << ins_->changeToOriginalID(road->raw_road->id, RoadMap) <<" ch: "<<
						carL->channel_id<<" from: "
						<< ins_->changeToOriginalID(road->raw_road->from, CrossMap) << " to: "
						<< ins_->changeToOriginalID(road->raw_road->to, CrossMap) << endl;
				}
			}
		}
#endif // FY_TEST
	}
}
void Solver::driveCarOnChannelToEndState(Road *road, int ch) {
	for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
		CarLocationOnRoad *carL = road->channel_carL[ch][cL];//车道内的车辆id及位置
		if (carL->state == STATE_terminated) { continue; }

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
			CarLocationOnRoad *prev_carL = road->channel_carL[ch][cL - 1];
			if (prev_carL->location - carL->location > speed) {//前车距离大于该车1个时间单位内可行驶的最大距离，视作无阻挡
				carL->location += speed;
				carL->state = STATE_terminated;
			}
			else {
				if (prev_carL->state == STATE_terminated) {
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
				else {
					if (prev_carL->state == STATE_terminated) {
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
}
void Solver::driveCarInGarage()
{
	int rsize = road_size * 2;
	for (int i = 0; i < rsize; ++i) {
		Road *road = topo.roads[i];
		if (road) {//假如有这样的路径
			sort(road->willOnRoad.begin(), road->willOnRoad.end(), car_id_sort);
			for (int j = 0; j < road->willOnRoad.size(); ++j) {//按照id顺序驶入道路
				CarLocationOnRoad * carLWillOnRoad = road->willOnRoad[j]; 
				Speed speed = min(road->raw_road->speed, ins_->raw_cars[carLWillOnRoad->car_id].speed);
				bool canDriveIn = false;
				for (int c = 0; c < road->channel_carL.size(); ++c) {
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
void Solver::recordProbOutCross(CarLocationOnRoad * carL, Cross * cross, Road * road)
{
	Turn turn =Front;
	if (carL->index + 1 < output_->routines[carL->car_id].roads.size()) {
		ID next_road_id = output_->routines[carL->car_id].roads[carL->index + 1];//这里之后可能会做修改
		Road *next_road = getNextRoad(carL, cross);
		if (next_road == NULL) {
			cout << "next_road is NULL \n";
			exit(1);
		}
		turn = topo.RoadTurn[road->raw_road->id][next_road_id];
	}
	
	carL->turn = turn;
	road->waitOutCarL.push_back(carL);
}

Road* Solver::getNextRoad(CarLocationOnRoad * carL, Cross * cross)
{
	Road *next_road = NULL;
	if (carL->index + 1 < output_->routines[carL->car_id].roads.size()) {
		ID next_road_id = output_->routines[carL->car_id].roads[carL->index + 1];//这里之后可能会做修改
		if (cross->raw_cross->id == ins_->raw_roads[next_road_id].from) {
			next_road = topo.roads[next_road_id];
		}
		else if (cross->raw_cross->id == ins_->raw_roads[next_road_id].to) {
			next_road = topo.roads[next_road_id + road_size];
		}
	}
	return next_road;
}

bool Solver::moveToNextRoad(Road * road, Road * next_road, CarLocationOnRoad * carL)
{
	if (carL->state == STATE_terminated)
		return false;
	Length S1 = road->raw_road->length - carL->location;//当前道路的可行驶距离
	if (next_road == NULL) {
		Speed speed = min(road->raw_road->speed, ins_->raw_cars[carL->car_id].speed);
		if (speed > S1) {
			carL->state = STATE_in_dst;
			road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());//第一个元素是最靠近路口的车辆
			inDst_num++;
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
				if (last_carL->state == STATE_terminated && last_carL->location > 1) {//车道内的车进入了终止状态，且有空位
					road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());//第一个元素是最靠近路口的车辆
					carL->location = min(last_carL->location - 1, V2 - S1);
					carL->channel_id = c;
					carL->state = STATE_terminated;
					carL->index += 1;
					next_road->channel_carL[carL->channel_id].push_back(carL);
					return true;
				}
				else if (last_carL->state == STATE_terminated && last_carL->location == 1
					&& c == (next_road->channel_carL.size() - 1)) {//这里需要重点检测下
					carL->location = road->raw_road->length;
					carL->state = STATE_terminated;
					return false;
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
    Length run_more_length = 0;
    for (auto road = shortest_paths[from][to].begin();
        road != shortest_paths[from][to].end(); ++road) {
        Length this_road_length = ins_->raw_roads[*road].length - run_more_length;
        if (this_road_length <= 0) {        // 跑得贼鸡儿快，都过了下一个路口了。
            run_more_length = 0;            // 如果道路长度大于道路最高限速，这种情况就不会发生。
            continue;
        }
        Speed this_road_speed = ins_->raw_roads[*road].speed;
        Speed next_road_speed = ins_->raw_roads[*(road + 1)].speed;
        Time this_road_cost_time = this_road_length / min(this_road_speed, car_speed);
        Length this_road_left_length =  // this_road_cost_time时间单位后，到达当前路口所剩余的距离。
            this_road_length - this_road_cost_time * min(this_road_speed, car_speed);
        run_more_length = min(next_road_speed, car_speed) > this_road_left_length ? // 在下一条道路上能够走的距离
            min(next_road_speed, car_speed) - this_road_left_length : 0;
        time_cost += (this_road_cost_time + 1);
    }
    return time_cost;
}


void Solver::init_solution_once() {
	int car_size = ins_->raw_cars.size();
	ID temp, next;
	Time  temp_time;
	Speed speed;
	//vector<Car> notPlanCar;
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
	std::cout << "the cost time is " << total_time << std::endl;
}
}