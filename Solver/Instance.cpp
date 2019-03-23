#include "Instance.h"

#include <fstream>
#include <regex>

using namespace std;

namespace codecraft2019 {

/*
 * ���뻷�����������������ļ���·����
 * �ɹ����������ļ�����true�����س�����false��
 */
bool Instance::load(Environment & env) {
    ifstream ifs;
    {/*************����road����****************/
        if (env.road_path.find("road") == string::npos)
            Log(Log::Warning) << "[Warning] RawRoad path may be wrong!" << endl;
        ifs.open(env.road_path);
        if (!ifs.is_open()) {
            Log(Log::Fatal) << "[Fatal] Can't open road.txt!" << endl;
            valid = false;
        } else {
            string line;
            getline(ifs, line);         // ��һ��ע�Ͳ�Ҫ��
            raw_roads.reserve(100);         // Ԥ�����ڴ棬��ֹpush_backʱ�ڴ治�㵼��vector������
            ID id, from, to;
            Length length;
            Speed speed;
            Channel channel;
            bool is_duplex;
            regex pattern(R"(.(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+).)");
            smatch result;
            while (!ifs.eof() && !ifs.bad()) {
                getline(ifs, line);
                regex_match(line, result, pattern);
                if (result.size() < 8) {
                    Log(Log::Warning) << "[Warning]\"" << line.data() << "\"" << "format error!" << endl;
                    continue;
                }
                id = stoi(result[1]);           // ����������Ͳ���int�Ļ���������Ҫǿ������ת����
                length = stoi(result[2]);
                speed = stoi(result[3]);
                channel = stoi(result[4]);
                from = stoi(result[5]);
                to = stoi(result[6]);
                is_duplex = static_cast<bool>(stoi(result[7]));
                raw_roads.push_back(move(RawRoad(
                    id,
                    length,
                    speed,
                    channel,
                    from,
                    to,
                    is_duplex
                )));
            }
        }
        ifs.close();
    }
    {/*************����car����****************/
        if (env.car_path.find("car") == string::npos)
            Log(Log::Warning) << "[Warning] RawCar path may be wrong!" << endl;
        ifs.open(env.car_path);
        if (!ifs.is_open()) {
            Log(Log::Fatal) << "[Fatal] Can't open car.txt" << endl;
            valid = false;
        } else {
            string line;
            getline(ifs, line);
            raw_cars.reserve(3000);
            ID id, from, to;
            Speed speed;
            Time plan_time;
            regex pattern(R"(.(\d+),\s*(\d+),\s*(\d+),\s*(\d+),\s*(\d+).)");
            smatch result;
            while (!ifs.eof() && !ifs.bad()) {
                getline(ifs, line);
                regex_match(line, result, pattern);
                if (result.size() < 6) {
                    Log(Log::Warning) << "[Warning]\"" << line.data() << "\"" << "format error!" << endl;
                    continue;
                }
                id = stoi(result[1]);
                from = stoi(result[2]);
                to = stoi(result[3]);
                speed = stoi(result[4]);
                plan_time = stoi(result[5]);
                raw_cars.push_back(move(RawCar(
					id,
					from,
					to,
                    speed,
                    plan_time
                )));
            }
        }
        ifs.close();
    }
    {/*************����cross����****************/
        if (env.cross_path.find("cross") == string::npos)
            Log(Log::Warning) << "[Warning] RawCross path may be wrong!" << endl;
        ifs.open(env.cross_path);
        if (!ifs.is_open()) {
            Log(Log::Fatal) << "[Fatal] Can't open cross.txt" << endl;
            valid = false;
        } else {
            string line;
            getline(ifs, line);
            raw_crosses.reserve(100);
            ID id, north, east, south, west;
            regex pattern(R"(.(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+).)");
            smatch result;
            while (!ifs.eof() && !ifs.bad()) {
                getline(ifs, line);
                regex_match(line, result, pattern);
                if (result.size() < 6) {
                    Log(Log::Warning) << "[Warning]\"" << line.data() << "\"" << "format error!" << endl;
                    continue;
                }
                id = stoi(result[1]);
                north = stoi(result[2]);
                east = stoi(result[3]);
                south = stoi(result[4]);
                west = stoi(result[5]);
                raw_crosses.push_back(move(RawCross(
					id,
                    north,
                    east,
                    south,
                    west
                )));
            }
        }
        ifs.close();
    }
    // �Ը���ID���������ӳ��
    vector<ID> roadId(raw_roads.size());
    vector<ID> crossId(raw_crosses.size());
    vector<ID> carId(raw_cars.size());
    for (int i = 0; i < raw_roads.size(); ++i)
        roadId[i] = raw_roads[i].id;
    for (int i = 0; i < raw_crosses.size(); ++i)
        crossId[i] = raw_crosses[i].id;
    for (int i = 0; i < raw_cars.size(); ++i)
        carId[i] = raw_cars[i].id;
    // ���������������ȴ�С��������
    sort(roadId.begin(), roadId.end());
    sort(crossId.begin(), crossId.end());
    sort(carId.begin(), carId.end());
    // ����ԭ��˳������ӳ��
    for (int i = 0; i < roadId.size(); ++i)
        road_map.toConsecutiveId(roadId[i]);
    for (int i = 0; i < crossId.size(); ++i)
        cross_map.toConsecutiveId(crossId[i]);
    for (int i = 0; i < carId.size(); ++i)
        car_map.toConsecutiveId(carId[i]);
    // ����Instance�е�ID
    for (int i = 0; i < raw_roads.size(); ++i) {
        raw_roads[i].id = road_map.toConsecutiveId(raw_roads[i].id);
        raw_roads[i].from = cross_map.toConsecutiveId(raw_roads[i].from);
        raw_roads[i].to = cross_map.toConsecutiveId(raw_roads[i].to);
    }
    for (int i = 0; i < raw_crosses.size(); ++i) {
        raw_crosses[i].id = cross_map.toConsecutiveId(raw_crosses[i].id);
        raw_crosses[i].north = raw_crosses[i].north == -1 ? -1 : road_map.toConsecutiveId(raw_crosses[i].north);
        raw_crosses[i].east = raw_crosses[i].east == -1 ? -1 : road_map.toConsecutiveId(raw_crosses[i].east);
        raw_crosses[i].south = raw_crosses[i].south == -1 ? -1 : road_map.toConsecutiveId(raw_crosses[i].south);
        raw_crosses[i].west = raw_crosses[i].west == -1 ? -1 : road_map.toConsecutiveId(raw_crosses[i].west);
        raw_crosses[i].road[0] = raw_crosses[i].north;
        raw_crosses[i].road[1] = raw_crosses[i].east;
        raw_crosses[i].road[2] = raw_crosses[i].south;
        raw_crosses[i].road[3] = raw_crosses[i].west;
    }
    for (int i = 0; i < raw_cars.size(); ++i) {
        raw_cars[i].id = car_map.toConsecutiveId(raw_cars[i].id);
        raw_cars[i].from = cross_map.toConsecutiveId(raw_cars[i].from);
        raw_cars[i].to = cross_map.toConsecutiveId(raw_cars[i].to);
    }
    // ��ʱ��һ��������ģ���Ҫ��������
    sort(raw_roads.begin(), raw_roads.end(), [](RawRoad &lhs, RawRoad &rhs) {return lhs.id < rhs.id; });
    sort(raw_crosses.begin(), raw_crosses.end(), [](RawCross &lhs, RawCross &rhs) {return lhs.id < rhs.id; });
    sort(raw_cars.begin(), raw_cars.end(), [](RawCar &lhs, RawCar &rhs) {return lhs.id < rhs.id; });
    return valid;
}

ID Instance::changeToZeroID(ID src,ID deta)
{
	return src - deta;
}

ID Instance::changeToOriginalID(ID src, ID deta)
{
	return src + deta;
}

}
