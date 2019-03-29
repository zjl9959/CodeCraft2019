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

bool pair_sort(const pair<Time,ID> &r1, const pair<Time, ID> &r2) {
	return r1.first < r2.first;
}

bool time_diff_sort(const pair<ID,int> &r1, const pair<ID, int> &r2) {
	return r1.second > r2.second;
}
bool car_time_id_sort(const CarLocationOnRoad *carL1, const CarLocationOnRoad *carL2) {//出发时间早，且id小的优先级高
	if (carL1->start_time < carL2->start_time)
		return true;
	else if(carL1->start_time == carL2->start_time)
		return carL1->car_id < carL2->car_id;
	return false;
}

// 查找id是否在路径中,如果在就返回true,否则返回false。
bool id_in_path(const List<ID> &path, const ID id) {
    for (List<ID>::const_iterator it = path.cbegin(); it != path.cend(); ++it) {
        if (id == *it)return true;
    }
    return false;
}
Time get_time(Instance *ins_, vector<ID> &roads, ID car_id) {
	double approxi_time = 0;
	for (int j = 0; j < roads.size(); ++j) {
		RawRoad *raw_road = &ins_->raw_roads[roads[j]];
		Speed speed = min(ins_->raw_cars[car_id].speed, raw_road->speed);
		approxi_time += (raw_road->speed * 1.0) / speed;
	}
	return ceil(approxi_time);
}
void Solver::run() {
    init();
	//init_solution();
    binary_generate_solution();
    //test_treeSearch();
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
	shortest_paths.reserve(topo.cross_size*topo.cross_size*topo.cross_size / 3);
	other_paths.reserve(topo.cross_size*topo.cross_size*topo.cross_size / 3);
	shortest_cross_paths.reserve(topo.cross_size*topo.cross_size*topo.cross_size/3);
	randseed = (int)time(NULL);
	//srand(randseed);
	srand(1553787816);
	Log(FY_TEST) << "the randseed is " << randseed << endl;
    shortest_paths.resize(topo.cross_size);
	other_paths.resize(topo.cross_size);
	shortest_cross_paths.resize(topo.cross_size);

    Log(Log::ZJL) << "\t------shortest path--------" << endl;
    Log(Log::ZJL) << "cross size:" << topo.cross_size << endl;
    for (ID i = 0; i < topo.cross_size; ++i) {
        shortest_paths[i].resize(topo.cross_size);
		shortest_cross_paths[i].resize(topo.cross_size);
		other_paths[i].resize(topo.cross_size);

        for (ID j = 0; j < topo.cross_size; ++j) {
            //Log(Log::ZJL) << "from " << i << " to " << j << ":";
            if (topo.pathID[i][j] != INVALID_ID) {
                ID temp = i;
				shortest_cross_paths[i][j].push_back(temp);
                while (temp != j && temp != INVALID_ID) {
                    ID next = topo.pathID[temp][j];
                    shortest_paths[i][j].push_back(topo.adjRoadID[temp][next]);
                    temp = next;
					shortest_cross_paths[i][j].push_back(temp);
                    //Log(Log::ZJL) << shortest_paths[i][j].back() << " ";
                }
            }
            //Log(Log::ZJL) << endl;
        }
    }
    // 其它数据的初始化。。。
}


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
		ID from = ins_->raw_cars[i].from, to = ins_->raw_cars[i].to;
		routine.roads = shortest_paths[from][to];
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
    stable_sort(run_time.begin(), run_time.end(), pair_sort);  // 在路上耗时少的车辆先出发。
    int binsearch_turn = 1;
	int car_num_mid = 1;
	vector<Time> start_times;
	int cnt = 0;
	clock_t a = clock();
    while (car_num_left <= car_num_right) {      // 二分调参大法
        Log(Log::ZJL) << "\t-----binary search turn " << binsearch_turn << "------" << endl;
		car_num_mid = car_num_left + ((car_num_right - car_num_left) / 2);
		Time current_time = changeTime(total_car_num, car_num_mid, run_time,start_times);
		aux.real_car_num = output_->routines.size();
		cnt++;
        Time cur_schedule_time = check_solution(output_->routines,aux);
		Log(Log::ZJL) << car_num_mid << endl;
        Log(Log::ZJL) << "current time:" << current_time << " schedule time:" << cur_schedule_time << endl;
        if (cur_schedule_time != -1) {                 // 解是合法的，说明可以提高car_num_mid
            car_num_left = car_num_mid + 1;
            if (cur_schedule_time < best_schdeule_time) {  // 记录最优解，这里check_solution能否传回准确的调度时间？
                best_schdeule_time = cur_schedule_time;
                best_start_times = start_times;
            }
        } else {
            car_num_right = car_num_mid - 1;
        }
    }
	clock_t b = clock();
	double dur = (b - a)*1.0 / CLOCKS_PER_SEC;
	Log(FY_TEST) << "times : " << cnt << "  " << dur<<endl;
	for (int i = 0; i < best_start_times.size(); ++i) {
		output_->routines[i].start_time = best_start_times[i];
	}
	
	//car_num_mid = 32;
	//Time current_time = changeTime(total_car_num, car_num_mid, run_time, start_times);
	generate_futher_solution();
	get_routines_cost_time();
	//aux.real_car_num = car_size;
	//check_solution(output_->routines, aux);
	
	//handle_deadLock();
	local_search();
	Log(FY_TEST) << "the latest time is" << latest_real_start_time <<" start time"<< latest_start_time<< endl;
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
void Solver::generate_futher_solution()
{
	vector<vector<Routine>> time_routine;
	time_routine.resize(20000);
	for (int i = 0; i < output_->routines.size(); ++i) {
		Time start_time = output_->routines[i].start_time;
		time_routine[start_time].push_back(output_->routines[i]);
	}
	aux.real_car_num = time_routine[1].size();

	Time time = check_solution(time_routine[1], aux);
	int temp_t;
	for (temp_t = 2; temp_t < time_routine.size(); ++temp_t) {
		if (time_routine[temp_t].size() > 0) {
			break;
		}
	}
	Time gap = temp_t - time +4;
	for (int i = 0; i < output_->routines.size(); ++i) {
		Time start_time = output_->routines[i].start_time;
		if (start_time > 1 && start_time -gap >= ins_->raw_cars[i].plan_time)
			output_->routines[i].start_time -= gap;
	}
	aux.real_car_num = output_->routines.size();
	Time end_time = check_solution(output_->routines, aux);
	Log(Log::ZJL) << end_time << endl;
}

Time Solver::handle_deadLock()
{
	/*vector<Routine> deadLock_routines;
	for (int i = 0; i < output_->routines.size(); ++i) {
		deadLock_routines.push_back(output_->routines[i]);
	}*/
	dead_lockCar.clear();
	clock_t a = clock();
	aux.real_car_num = output_->routines.size();
	Time time = check_solution(output_->routines, aux);
	while(time == -1) {
		for (int i = 0; i < dead_lockCar.size(); ++i) {
			ID old_id ,car_id = dead_lockCar[i].first;
			output_->routines[car_id].roads.clear();
			Car *car = topo.cars[car_id];
			output_->routines[car_id].roads = topo.Dijkstra(car->raw_car->from, car->raw_car->to, dead_lockCar[i].second);
			//priority_first_search(deadLock_routines[dead_lockCar[i]].start_time, topo.cars[dead_lockCar[i]], deadLock_routines[dead_lockCar[i]].roads);
		}

		time = check_solution(output_->routines, aux);
		//Log(Log::ZJL) << time << endl;

	}
	clock_t b = clock();
	double dur = (b - a)*1.0 / CLOCKS_PER_SEC;
	Log(FY_TEST) << "run time is" << dur << endl;
	Log(FY_TEST) << time << endl;
	return time;
}

void Solver::start_early(List<Routine> &temp_routines,int d1,int d2)
{
	Time new_time;
	Log(FY_TEST) << "--------------early start--------------\n";
	int cnt = 0;
	for (int i = 0; i < temp_routines.size(); ++i) {
		Routine *routine = &temp_routines[i];
		if (routine->start_time > 350)
		{
			cnt++;
		}
	}
	if (cnt > 100) {
		for (int i = 0; i < output_->routines.size(); ++i) {
			Routine *routine = &output_->routines[i];
			if (routine->start_time <= 60) {
				routine->start_time = max(ins_->raw_cars[i].plan_time, routine->start_time - 60);
			}
			if (routine->start_time > 60 && routine->start_time < 100)
			{
				routine->start_time = routine->start_time - d1;
			}
			else if (routine->start_time >= 100) {
				routine->start_time -= d1;
			}
		}
	}
	else {
		for (int i = 0; i < output_->routines.size(); ++i) {
			Routine *routine = &output_->routines[i];
			if (routine->start_time <= 60) {
				routine->start_time = max(ins_->raw_cars[i].plan_time, routine->start_time -  60);
			}
			if (routine->start_time > 60 && routine->start_time < 100)
			{
				routine->start_time = routine->start_time - d2;
			}
			else if (routine->start_time >= 100) {
				routine->start_time -= d2;
			}
			//Log(FY_TEST) << "the jian is" << d2<<endl;
		}
	}
	new_time = check_solution(output_->routines, aux);
	if (new_time == -1) {
		new_time = handle_deadLock();
	}
	if (new_time < current_time) {
		temp_routines.clear();
		for (int i = 0; i < output_->routines.size(); ++i) {
			temp_routines.push_back(output_->routines[i]);
		}
		Log(FY_TEST) << "the new time is :" << new_time << endl;
		current_time = new_time;
	}
}

void Solver::insert_to_early(List<Routine> &temp_routines)
{
	Time new_time;
	int K = 10;
	Log(FY_TEST) << "insert_to_early" << endl;
	for (int m = 0; m < 10; ++m) {
		partial_sort(time_diff.begin(), time_diff.begin() + K, time_diff.end(), time_diff_sort);

		for (int k = 0; k < K; ++k) {//对车辆进行近似评估
			ID car_id = time_diff[k].first;
			Time start_time = output_->routines[car_id].start_time;
			Car *car = topo.cars[car_id];
			vector<ID> roads;
			if(output_->routines[car_id].start_time>150)
				output_->routines[car_id].start_time = output_->routines[car_id].start_time - rand() % 150 + 10;

			if (priority_first_search(output_->routines[car_id].start_time, car, roads) != -1) {
				output_->routines[car_id].roads.clear();
				output_->routines[car_id].roads = roads;
				output_->routines[car_id].cost_time = get_time(ins_, roads, car_id);

			}
		}
		new_time = check_solution(output_->routines, aux);
		if (new_time == -1) {
			new_time = handle_deadLock();
		}
		if (new_time < current_time) {
			temp_routines.clear();
			for (int i = 0; i < output_->routines.size(); ++i) {
				temp_routines.push_back(output_->routines[i]);
			}
			Log(FY_TEST) << "the new time is :" << new_time << endl;
			current_time = new_time;
		}
	}
}

void Solver::local_search()
{
	aux.real_car_num = car_size;
	current_time = check_solution(output_->routines, aux);
	vector<Routine> temp_routines;

	temp_routines.clear();
	for (int i = 0; i < output_->routines.size(); ++i) {
		cars_cost_time[i].second -= output_->routines[i].cost_time;
		temp_routines.push_back(output_->routines[i]);
	}
	int K = 10;
	Time new_time;
	find_newPath_and_time(temp_routines,cars_cost_time,Cost_time);
	find_newPath_and_time(temp_routines, time_diff, Latest_Time);
	start_early(temp_routines,50,35);
	
	//for (int m = 0; m < 50; ++m) {
	//	partial_sort(time_diff.begin(), time_diff.begin() + 10, time_diff.end(), time_diff_sort);
	//	for (int k = 0; k < 10; ++k) {//对车辆进行近似评估
	//		ID car_id = time_diff[k].first;
	//		Time start_time = output_->routines[car_id].start_time;
	//		Car *car = topo.cars[car_id];
	//		vector<ID> roads;
	//		Time time = find_best_start_time(car, output_->routines[car_id], roads);
	//		if (time != -1) {
	//			output_->routines[car_id].start_time = time;
	//			Log(FY_TEST) << "\nstart time is :" << time << endl;
	//			output_->routines[car_id].roads.clear();
	//			output_->routines[car_id].roads = roads;
	//			output_->routines[car_id].cost_time = get_time(ins_, roads, car_id);
	//		}
	//	}
	//	new_time = check_solution(output_->routines, aux);
	//	if (new_time == -1) {
	//		new_time = handle_deadLock();
	//	}
	//	if (new_time < current_time) {
	//		temp_routines.clear();
	//		for (int i = 0; i < output_->routines.size(); ++i) {
	//			temp_routines.push_back(output_->routines[i]);
	//		}
	//		Log(FY_TEST) << "the next new time is :" << new_time << endl;
	//		current_time = new_time;
	//	}
	//}
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 30, 350);
	find_newPath_and_time(temp_routines, time_diff, Latest_Time, 30, 350);
	start_early(temp_routines, 30, 0);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);
	start_early(temp_routines, 30, 1);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);
	start_early(temp_routines, 30, 10);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);
	find_newPath_and_time(temp_routines, time_diff, Cost_time, 30, 320);
	/*insert_to_early(temp_routines);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);
	insert_to_early(temp_routines);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 20, 350);*/

	start_early(temp_routines, 30, 40);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 30, 500);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 30, 500);
	find_newPath_and_time(temp_routines, cars_cost_time, Cost_time, 30, 500);

	output_->routines.clear();
	int cnt = 0;
	for (int i = 0; i < temp_routines.size(); ++i) {
		output_->routines.push_back(temp_routines[i]);
		if (output_->routines[i].start_time > 350) {
			cnt++;
		}
	}
	new_time = check_solution(output_->routines, aux);
	cout << "the end time is :" << new_time <<" \n the too late car_num is "<<cnt<< endl;
}
void Solver::find_newPath_and_time(vector<Routine> &temp_routines, List<std::pair<ID, int>> &neighbour, Neighbour neigh,int K, Time my_min,Time my_max)
{
	Time new_time;
	for (int m = 0; m < 100; ++m) {
		if (neigh == Cost_time) {
			for (int i = 0; i < output_->routines.size(); ++i) {
				neighbour[i].second -= output_->routines[i].cost_time;
			}
		}
		partial_sort(neighbour.begin(), neighbour.begin() + K, neighbour.end(), time_diff_sort);

		for (int k = 0; k < K; ++k) {//对车辆进行近似评估
			ID car_id = neighbour[k].first;
			Time start_time = output_->routines[car_id].start_time;
			Car *car = topo.cars[car_id];
			vector<ID> roads;

			if (output_->routines[car_id].start_time >= my_min && output_->routines[car_id].start_time<= my_max )
				output_->routines[car_id].start_time = output_->routines[car_id].start_time - rand() % 130 - 50;

			if (priority_first_search(output_->routines[car_id].start_time, car, roads) != -1) {
				output_->routines[car_id].roads.clear();
				output_->routines[car_id].roads = roads;
				output_->routines[car_id].cost_time = get_time(ins_, roads, car_id);

			}
		}
		new_time = check_solution(output_->routines, aux);
		if (new_time == -1) {
			new_time = handle_deadLock();
		}
		if (new_time < current_time) {
			temp_routines.clear();
			for (int i = 0; i < output_->routines.size(); ++i) {
				temp_routines.push_back(output_->routines[i]);
			}
			Log(FY_TEST) << "the new time is :" << new_time << endl;
			current_time = new_time;
		}
	}
}

Time Solver::find_best_start_time(Car *car, Routine &routine, vector<ID> &best_roads) {
	if (routine.start_time < 350) return -1;
	Time best_cost_time = INT_MAX, best_start_time = -1,temp;
	Time time = routine.start_time - 50 - rand() % 10;
	for (int t = car->raw_car->plan_time; t <= time; ++t) {
		vector<ID> temp_roads;
		temp = priority_first_search(t, car, temp_roads);
		if ( temp!=-1 && temp < best_cost_time) {
			best_cost_time = temp;
			best_roads = temp_roads;
			best_start_time = t;
		}
	}
	return best_start_time;
}
Time Solver::changeTime(int total_car_num,int car_num_mid, vector<pair<Time, ID>> &run_time, vector<Time> &start_times)
{
	
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
		stable_sort(cars_run_time.begin(), cars_run_time.end(), pair_sort);
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


int Solver::check_solution(const vector<Routine> &routines,Aux &aux, Mode mode)
{
	vector<vector<ID>> time_car;
	time_car.resize(100000);
    timeslice.clear();
	time_road_condition.clear();
	time_diff.clear();
	cars_cost_time.clear();
	if(mode == GetZeroSpeed)
		zero_speed_car.clear();
	for (int i = 0; i < routines.size(); ++i) {
		Time start_time = routines[i].start_time;
		if(start_time !=-1)
			time_car[start_time].push_back(i);
	}
	inDst_num = 0;
	cars_totalTime = 0;
	time_diff.resize(car_size);
	cars_cost_time.resize(car_size);
	/*aux.roadNullNumOfTime.clear();
	aux.type_numTime.clear();
	aux.type_numTime.resize(2000);
	for (int i = 0; i < aux.type_numTime.size(); ++i) {
		aux.type_numTime[i].resize(aux.type_num);
	}*/
	for ( t = 0; t <= MAX_TIME; t++) {

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
				for (int k = 0; k < cross->road.size(); ++k) {
					Road *road = cross->road[k];//如何判断冲突
					bool conflict = false;

					int old_size = road->waitOutCarL.size();
					for (int r = 0; r < road->waitOutCarL.size(); ++r) {//按照优先级顺序依次遍历
						CarLocationOnRoad *carL = road->waitOutCarL[r];
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
						if (moveToNextRoad(road, next_road, carL,routines) ||
							carL->state == STATE_terminated) {//说明在该时刻已经不可能出路口
							road->waitOutCarL.erase(road->waitOutCarL.begin());
							driveCarOnChannelToEndState(road, temp_ch);
							canSchedule = true;
							r = -1;
						}
						else {//第一优先级不能出路口，那么后面的车都不能出
							break;
						}
					}
				}
			}
			if (!canSchedule)
				break;
		}

		bool isvalid = true;
		for (int c = 0; c < cross_size; ++c) {
			Cross *cross = topo.crosses[c];
			for (int k = 0; k < cross->road.size(); ++k) {
				Road *road = cross->road[k];
				if (road->waitOutCarL.size() > 0) {
					dead_lockCar.push_back(make_pair(road->waitOutCarL[0]->car_id,road->raw_road->id));
					isvalid = false;
				}
			}
		}
		if (!isvalid) {
			clearRoadVector(routines.size());
			return -1;
		}
		if (inDst_num == aux.real_car_num) {
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
			new_carL->real_speed = 0;
			if (ins_->raw_roads[first_road].from == ins_->raw_cars[car_id].from) {
				topo.roads[first_road]->willOnRoad.push_back(new_carL);
			}
			else {
				topo.roads[first_road +road_size]->willOnRoad.push_back(new_carL);
			}
		}
		driveCarInGarage();
		///记录路况信息
		vector<vector<RoadCondition>> road_conditions;
		road_conditions.resize(cross_size);
		for (int i = 0; i < cross_size; ++i) {
			road_conditions[i].resize(cross_size);
		}
		for (int i = 0; i < road_size * 2; ++i) {
			Road *road = topo.roads[i];
			int num = 0;
			if (road) {
				double ratio_sum = 0;
				int cnt = 0;
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
					if (road->channel_carL[ch].size() == 0)
						num += road->raw_road->length;
					else {
						CarLocationOnRoad *carL = road->channel_carL[ch].back();
						num += carL->location - 1;
					}
					for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
						CarLocationOnRoad *carL = road->channel_carL[ch][cL];
						cnt++;
						ratio_sum += (carL->real_speed*1.0) / ins_->raw_cars[carL->car_id].speed;
						if (mode == GetZeroSpeed) {
							if (carL->real_speed == 0) {
								zero_speed_car.push_back(make_pair(carL->car_id, road->raw_road->id));
							}
						}
					}
				}
				if (cnt == 0)
					road_conditions[road->from_id][road->to_id].avg_speed_ratio = 1;
				else
					road_conditions[road->from_id][road->to_id].avg_speed_ratio = ratio_sum / (cnt*1.0);
				road_conditions[road->from_id][road->to_id].vacancy_num = num;
			}
		}
		if (mode == GetZeroSpeed) {
			if (zero_speed_car.size() >= 8) {
				clearRoadVector(routines.size());
				return -2;
			}
			else {
				zero_speed_car.clear();
			}
		}
		
		time_road_condition.push_back(road_conditions);
#if FY_TEST == 0
		//Log(FY_TEST) << "\n\n------------------- Time  " << t << "-------------------\n" << endl;
		/*vector<vector<int>> road_nullNum;
		road_nullNum.resize(cross_size);
		for (int i = 0; i < cross_size; ++i) {
			road_nullNum[i].resize(cross_size);
		}
		for (int i = 0; i < road_size*2; ++i) {
			Road *road = topo.roads[i];
			int num = 0;
			if (road) {
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
					if (road->channel_carL[ch].size() == 0)
						num += road->raw_road->length;
					else {
						CarLocationOnRoad *carL = road->channel_carL[ch].back();
						num += carL->location - 1;
					}
				}
				road_nullNum[road->from_id][road->to_id] = num;
			}
		}
		aux.roadNullNumOfTime.push_back(road_nullNum);*/

		
		///可视化
		vector<vector<CarInfo>> road_carInfos;
		road_carInfos.resize(road_size);
		for (int i = 0; i < road_size ; ++i) {
			Road *road = topo.roads[i];
			vector<CarInfo> carInfos; //可视化信息
			for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
				for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
					CarLocationOnRoad *carL = road->channel_carL[ch][cL];
					CarInfo carinfo(carL->car_id,carL->location,road->raw_road->channel -1 - carL->channel_id,carL->state);
					
					carInfos.push_back(move(carinfo));
				}
			}
			if (road->raw_road->is_duplex) {
				Road *road = topo.roads[i + road_size];
				for (int ch = 0; ch < road->channel_carL.size(); ++ch) {
					for (int cL = 0; cL < road->channel_carL[ch].size(); ++cL) {
						CarLocationOnRoad *carL = road->channel_carL[ch][cL];
						CarInfo carinfo(carL->car_id, carL->location, road->raw_road->channel +carL->channel_id, carL->state);
						
						carInfos.push_back(move(carinfo));
					}
				}
			}
			road_carInfos[i] = carInfos;
		}
		timeslice.push_back(road_carInfos);

		///可视化

		///信息打印
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
				carL->real_speed = speed;
				carL->state = STATE_terminated;
			}
		}
		else {
			CarLocationOnRoad *prev_carL = road->channel_carL[ch][cL - 1];
			if (prev_carL->location - carL->location > speed) {//前车距离大于该车1个时间单位内可行驶的最大距离，视作无阻挡
				carL->location += speed;
				carL->real_speed = speed;
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
					carL->real_speed = speed;
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
					carL->real_speed = speed;
					carL->state = STATE_terminated;
				}
			}
			else {
				CarLocationOnRoad *prev_carL = road->channel_carL[c][cL - 1];
				if (prev_carL->location - carL->location > speed) {//前车距离大于该车1个时间单位内可行驶的最大距离，视作无阻挡
					carL->location += speed;
					carL->real_speed = speed;
					carL->state = STATE_terminated;
				}
				else 
				{
					if (prev_carL->state == STATE_terminated) {
						carL->real_speed = prev_carL->location - carL->location - 1;
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
						carLWillOnRoad->real_speed = speed;
						road->channel_carL[c].push_back(carLWillOnRoad);

						latest_start_time = output_->routines[carLWillOnRoad->car_id].start_time;
						latest_real_start_time = t;
						time_diff.push_back(make_pair(carLWillOnRoad->car_id, latest_real_start_time));

						canDriveIn = true;
						break;
					}
					else {
						CarLocationOnRoad *carL = road->channel_carL[c].back();
						if (carL->location > 1) {
							speed = min(carL->location - 1, speed);
							carLWillOnRoad->real_speed = speed;
							carLWillOnRoad->channel_id = c;
							carLWillOnRoad->location = speed;
							road->channel_carL[c].push_back(carLWillOnRoad);

							latest_start_time = output_->routines[carLWillOnRoad->car_id].start_time;
							latest_real_start_time = t;
							time_diff.push_back(make_pair(carLWillOnRoad->car_id, latest_real_start_time));

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
	ID next_road_id;
	if (carL->index + 1 < routines[carL->indexInRoutine].roads.size()) {
		next_road_id = routines[carL->indexInRoutine].roads[carL->index + 1];//这里之后可能会做修改
		turn = topo.RoadTurn[road->raw_road->id][next_road_id];
	}
	
	carL->turn = turn;
	if (turn == InvalidTurn) {
		Log(FY_TEST) << "  the wrong path is"<<endl;

		for (int i = 0; i < routines[carL->indexInRoutine].roads.size(); ++i) {
			Log(FY_TEST) << routines[carL->indexInRoutine].roads[i] << "  ";
		}
		Log(FY_TEST) <<endl<<carL->car_id <<" from "<<road->raw_road->id<<" to "<< next_road_id <<" turn error";
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

bool Solver::moveToNextRoad(Road * road, Road * next_road, CarLocationOnRoad * carL, const vector<Routine> &routines)
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
			cars_totalTime += t - output_->routines[carL->car_id].start_time;
			cars_cost_time[carL->car_id] = make_pair(carL->car_id,t - output_->routines[carL->car_id].start_time);
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
			carL->real_speed = road->raw_road->length - carL->location - 1;
			carL->location = road->raw_road->length;
			carL->state = STATE_terminated;
			return false;
		}
		for (int c = 0; c < next_road->channel_carL.size(); ++c) {//遍历下一条路的所有车道
			if (next_road->channel_carL[c].size() == 0) {
				road->channel_carL[carL->channel_id].erase(road->channel_carL[carL->channel_id].begin());
				carL->real_speed = V2;
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
						carL->real_speed = min(last_carL->location - 1 + S1, V2);
						carL->location = min(last_carL->location - 1, V2 - S1);
						carL->channel_id = c;
						carL->state = STATE_terminated;
						carL->index += 1;
						next_road->channel_carL[carL->channel_id].push_back(carL);
						return true;
					}
					else if (last_carL->location == 1 && c == (next_road->channel_carL.size() - 1)) {
						carL->real_speed = road->raw_road->length - carL->location - 1;
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
						carL->real_speed = V2;
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
	//rauxs.reserve(car_size);
	//for (int i = 0; i < car_size; ++i) {   // 每辆车都走最短路径
	//	Routine routine;
	//	routine.car_id = ins_->raw_cars[i].id;
	//	routine.roads = shortest_paths[ins_->raw_cars[i].from][ins_->raw_cars[i].to];
	//	routine.crosses = shortest_cross_paths[ins_->raw_cars[i].from][ins_->raw_cars[i].to];
	//	Speed car_speed = ins_->raw_cars[i].speed;
	//	Time cost_time =0;
	//	for (int i = 0; i < routine.roads.size(); ++i) {
	//		RawRoad *raw_road = &ins_->raw_roads[routine.roads[i]];
	//		Speed speed = min(car_speed, raw_road->speed);
	//		cost_time += raw_road->length / speed;
	//	}
	//	RAux raux(cross_size,cost_time,&ins_->raw_cars[i],routine.car_id);
	//	rauxs.push_back(move(raux));
	//	output_->routines.push_back(move(routine));
	//}
	//vector<vector<RAux>> Rtypes;
	//for (int i = 0; i < output_->routines.size(); ++i) {
	//	Routine *routine = &output_->routines[i];
	//	for (int j = 0; j < routine->crosses.size(); ++j) {
	//		for (int k = j+1; k < routine->crosses.size(); ++k) {
	//			rauxs[i].subPath[routine->crosses[j]][routine->crosses[k]] = true;
	//		}
	//	}
	//	/*for (int j = 0; j < topo.crosses.size(); ++j) {
	//		for (int k = 0; k < topo.crosses.size(); ++k) {
	//			if (rauxs[i].subPath[j][k] == true)
	//				cout << "from :" << j << " to: " << k << endl;
	//		}
	//	}
	//	cout << endl;*/
	//}
	//stable_sort(rauxs.begin(), rauxs.end(), raux_sort);//按照消耗时间排序

	//for (int i = 0; i < rauxs.size(); ++i) {
	//	ID from = rauxs[i].raw_car->from, to = rauxs[i].raw_car->to;
	//	bool notfind = true;
	//	for (int t = 0; t < Rtypes.size() && notfind; ++t) {
	//		for (int j = 0; j < Rtypes[t].size(); ++j) {
	//			ID from2 = Rtypes[t][j].raw_car->from, to2 = Rtypes[t][j].raw_car->to;
	//			if (Rtypes[t][j].subPath[from][to] || rauxs[i].subPath[from2][to2]){
	//				//|| Rtypes[t][j].subPath[to][from] || rauxs[i].subPath[to2][from2]) {
	//				Rtypes[t].push_back(rauxs[i]);
	//				notfind = false;
	//				break;
	//			}
	//		}
	//	}
	//	if (notfind) {
	//		vector<RAux> rtype;
	//		rtype.push_back(rauxs[i]);
	//		Rtypes.push_back(rtype);
	//	}
	//}
	///*for (int i = 0; i < Rtypes.size(); ++i) {
	//	Log(FY_TEST) << "type " << i << endl;
	//	for (int j = 0; j < Rtypes[i].size(); ++j) {
	//		Log(FY_TEST) << Rtypes[i][j].car_id<<"  ";
	//	}
	//	Log(FY_TEST) << endl;

	//}*/
	//aux.type_num = Rtypes.size();

	//int max_car_num = 30;
	//int max_type_num = 20;
	////for (int t = 0; t < MAX_TIME; ++t) {
	//vector<Routine> temp_routines;
	//int car_onRoadNum = 0;
	//int type_onRoadnum = 0;
	//for (int i = 0; i < Rtypes.size(); ++i) {
	//	int cnt = 0;
	//	for (int j= 0; j<Rtypes[i].size(); ++j) {
	//		ID car_id = Rtypes[i][j].car_id;
	//		if (cnt > max_car_num) break;
	//		cnt++;
	//		car_onRoadNum++;
	//		output_->routines[car_id].start_time = ins_->raw_cars[car_id].plan_time;
	//		temp_routines.push_back(output_->routines[car_id]);
	//		Rtypes[i].erase(Rtypes[i].begin());
	//		j = -1;
	//	}
	//	type_onRoadnum++;
	//	if (type_onRoadnum > max_type_num) {
	//		break;
	//	}
	//}
	//Time t_time = check_solution(temp_routines, aux);
	//Time prev_t_time;
	//cout << "the cost time is  :" << t_time << " the total num is" << car_onRoadNum << endl;

	///*for (int i = 0; i < Rtypes.size(); ++i) {
	//	if (Rtypes[i].size() > 0) {
	//		int cnt = 0;
	//		for (int j = 0; j < Rtypes[i].size(); ++j) {
	//			ID car_id = Rtypes[i][j].car_id;
	//			Routine *routine = &output_->routines[car_id];
	//			RawRoad *raw_road = &ins_->raw_roads[routine->roads[0]];
	//			 if (cnt > 3) break;
	//			double road_position_num = raw_road->channel*raw_road->length *(4.0)/5.0;
	//			for (int t = ins_->raw_cars[car_id].plan_time; t < t_time; ++t) {
	//				if (aux.roadNullNumOfTime[t][routine->crosses[0]][routine->crosses[1]] > road_position_num)
	//				{
	//					routine->start_time = t;
	//					break;
	//				}
	//			}
	//			temp_routines.push_back(move(output_->routines[car_id]));

	//			Rtypes[i].erase(Rtypes[i].begin());
	//			j = -1;
	//			cnt++;
	//			car_onRoadNum++;
	//		}
	//		
	//	}
	//}
	//t_time = check_solution(temp_routines, aux);
	//Log(FY_TEST) << "the cost time is  :" << t_time << " the total num is" << car_onRoadNum << endl;*/
	//
	//cout << "\n\n";
	//int temp = 0;
	//while (temp < 10) {
	//	type_onRoadnum = 0;
	//	for (int i = 0; i < Rtypes.size(); ++i) {
	//		if (Rtypes[i].size() > 0) {
	//			int cnt = 0;
	//			for (int j = 0; j < Rtypes[i].size(); ++j) {
	//				ID car_id = Rtypes[i][j].car_id;
	//				Routine *routine = &output_->routines[car_id];
	//				RawRoad *raw_road = &ins_->raw_roads[routine->roads[0]];
	//				if (cnt > 10) break;
	//				double road_position_num = raw_road->channel*raw_road->length *(4.0) / 5.0;
	//				bool not_find = true;
	//				for (int t = ins_->raw_cars[car_id].plan_time; t < t_time && not_find; ++t) {
	//					if (aux.roadNullNumOfTime[t][routine->crosses[0]][routine->crosses[1]] > road_position_num)
	//					{
	//						routine->start_time = t;
	//						not_find = false;
	//						break;
	//					}
	//				}
	//				if (not_find) break;
	//				temp_routines.push_back(output_->routines[car_id]);

	//				Rtypes[i].erase(Rtypes[i].begin());
	//				j = -1;
	//				cnt++;
	//				car_onRoadNum++;
	//			}
	//			type_onRoadnum++;
	//			if (type_onRoadnum > 15)break;
	//		}
	//	}
	//	t_time = check_solution(temp_routines, aux);
	//	Log(FY_TEST) << "the cost time is  :" << t_time << " the total num is" << car_onRoadNum << endl;
	//	temp++;
	//	if (temp == 11) {
	//		prev_t_time = t_time;
	//	}
	//}
	//
	//while (car_onRoadNum < output_->routines.size())
	//{
	//	cout << "\n\n";
	//	int max_numPerType = 15;
	//	int max_type_num = 20;
	//	if (output_->routines.size() - car_onRoadNum < 1200) {
	//		max_numPerType = 80;
	//		max_type_num = 40;
	//	}
	//	int temp = 0;
	//	while (temp < 10) {
	//		if (car_onRoadNum >= output_->routines.size())break;
	//		type_onRoadnum = 0;
	//		for (int i = 0; i < Rtypes.size(); ++i) {
	//			if (Rtypes[i].size() > 0) {
	//				int cnt = 0;
	//				bool not_find = true;
	//				for (int j = 0; j < Rtypes[i].size(); ++j) {
	//					ID car_id = Rtypes[i][j].car_id;
	//					Routine *routine = &output_->routines[car_id];
	//					RawRoad *raw_road = &ins_->raw_roads[routine->roads[0]];
	//					if (cnt > max_numPerType) break;
	//					double road_position_num = raw_road->channel*raw_road->length *(4.0) / 5.0;
	//					bool not_find_in = true;
	//					for (int t = prev_t_time; t < t_time && not_find_in; ++t) {
	//						if (aux.roadNullNumOfTime[t][routine->crosses[0]][routine->crosses[1]] > road_position_num)
	//						{
	//							routine->start_time = t;
	//							not_find_in = false;
	//							not_find = false;
	//							break;
	//						}
	//					}
	//					if (not_find_in) break;
	//					temp_routines.push_back(output_->routines[car_id]);

	//					Rtypes[i].erase(Rtypes[i].begin());
	//					j = -1;
	//					cnt++;
	//					car_onRoadNum++;
	//				}
	//				if(!not_find)
	//					type_onRoadnum++;
	//				if (type_onRoadnum > max_type_num)break;
	//			}
	//		}
	//		t_time = check_solution(temp_routines, aux);
	//		cout << "the cost time is  :" << t_time << " the total num is" << car_onRoadNum << endl;
	//		temp++;
	//		if (temp == 10) {
	//			prev_t_time = t_time - 100;
	//		}
	//	}
	//}
}


void Solver::get_routines_cost_time()
{
	Time max_cost_time =-1;
	for (int i = 0; i < output_->routines.size(); ++i) {
		Routine *routine = &output_->routines[i];
		routine->cost_time = get_time(ins_, routine->roads, routine->car_id);
		if (max_cost_time < routine->cost_time)
			max_cost_time = routine->cost_time;
	}
	Log(FY_TEST) << max_cost_time << endl;
}

void Solver::test_treeSearch() {
    bool log_out = true;
    ID car = 1;
    aux.real_car_num = car_size;
    Time time = check_solution(output_->routines, aux);
    int count = 0;
    clock_t start = clock();
    for (; car < car_size; ++car) {
        Log(log_out) << "car:" << car << endl;
        Time lb = min_time_cost(car, ins_->raw_cars[car].from, ins_->raw_cars[car].to);
        Log(log_out) << "time low bound:" << lb << endl;
        List<ID> path;
        Time t = priority_first_search(output_->routines[car].start_time, topo.cars[car], path);
        Log(log_out) << "time:" << t << endl;;
        for (auto it = path.begin(); it != path.end(); ++it) {
            Log(log_out) << *it << " ";
        }
        if (t == -1)
            count++;
    }
    cout << "total num:" << car_size << endl;
    cout << "failed num:" << count << endl;
    cout << "used time:" << clock() - start << endl;
}

// 输入：车辆出发时间；车辆ID；最优路径的保存向量。
// 输出：车辆在最优路径上的估计行驶时间。
// 在给定的出发时间下，根据实际路况为单个车辆规划最优行驶路径。
Time Solver::priority_first_search(Time start_time, Car * car, List<ID> &roads)
{
    const int max_iter = INT_MAX;
    const int time_slice_num = time_road_condition.size();
    const ID from = car->from->raw_cross->id;
    const ID to = car->to->raw_cross->id;
    int iter_num = 0;
    double best_obj = 1000000.0;
    List<ID> best_sol;
    List<ID> cur_sol = { from };
    multimap<double, List<ID>> pqueue;
    pqueue.insert(make_pair(0, cur_sol));
    while (!pqueue.empty() && iter_num < max_iter) {
        double cur_obj = pqueue.begin()->first;
        cur_sol.swap(pqueue.begin()->second);
        pqueue.erase(pqueue.begin());
        ID cur_cross = cur_sol.back();
        if (Math::strongLess(best_obj, cur_obj +
            min_time_cost(car->raw_car->id, cur_cross, to))) {                    // 剪枝
            continue;
        }
        for (auto it = topo.crosses[cur_cross]->inCrossRoad.begin();
            it != topo.crosses[cur_cross]->inCrossRoad.end(); ++it) {
            ID next_cross = (*it)->to_id;
            if (!id_in_path(cur_sol,next_cross)) {                              // 下一个路口不在路径中
                ID road = (*it)->raw_road->id;
                Length road_length = ins_->raw_roads[road].length;
                Speed max_speed = min(car->raw_car->speed, ins_->raw_roads[road].speed);
                Time time_point = start_time + Math::dt5oi(cur_obj);
                if (time_point >= time_slice_num)continue;
                RoadCondition condition(time_road_condition[time_point][cur_cross][next_cross]);
                if (Math::weakEqual(condition.avg_speed_ratio, 0.0)) {
                    continue;
                }
                double road_speed_time =
                    (double)road_length / ((double)max_speed*condition.avg_speed_ratio);
                if (road_speed_time < 0)
                    continue;
                double next_obj = cur_obj + road_speed_time;
                cur_sol.push_back(next_cross);
                List<ID> next_sol(cur_sol);
                cur_sol.pop_back();
                if (next_cross == to && Math::strongLess(next_obj, best_obj)) { // 此时找到一个更好的完整的解
                    best_obj = next_obj;
                    best_sol = next_sol;
                    continue;                                                   // 不用再扩展完整解
                }
                pqueue.insert(make_pair(next_obj, move(next_sol)));
            }
        }
        ++iter_num;
    }
    if (best_sol.empty() || best_sol.back() != to)
        return -1;
	for (int i = 0; i < best_sol.size()-1; ++i) {
		roads.push_back(topo.adjRoadID[best_sol[i]][best_sol[i + 1]]);
	}
    return Math::dt5oi(best_obj);
}

}
