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
            Log(Log::Warning) << "[Warning] Road path may be wrong!" << endl;
        ifs.open(env.road_path);
        if (!ifs.is_open()) {
            Log(Log::Fatal) << "[Fatal] Can't open road.txt!" << endl;
            valid = false;
        } else {
            string line;
            getline(ifs, line);         // ��һ��ע�Ͳ�Ҫ��
            roads.reserve(100);         // Ԥ�����ڴ棬��ֹpush_backʱ�ڴ治�㵼��vector������
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
                roads.push_back(move(Road(
                    changeToZeroID(id,RoadMap),
                    length,
                    speed,
                    channel,
                    changeToZeroID(from,CrossMap),
                    changeToZeroID(to,CrossMap),
                    is_duplex
                )));
            }
        }
        ifs.close();
    }
    {/*************����car����****************/
        if (env.car_path.find("car") == string::npos)
            Log(Log::Warning) << "[Warning] Car path may be wrong!" << endl;
        ifs.open(env.car_path);
        if (!ifs.is_open()) {
            Log(Log::Fatal) << "[Fatal] Can't open car.txt" << endl;
            valid = false;
        } else {
            string line;
            getline(ifs, line);
            cars.reserve(3000);
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
                cars.push_back(move(Car(
					changeToZeroID(id, CarMap),
					changeToZeroID(from, CrossMap),
					changeToZeroID(to, CrossMap),
                    speed,
                    plan_time
                )));
            }
        }
        ifs.close();
    }
    {/*************����cross����****************/
        if (env.cross_path.find("cross") == string::npos)
            Log(Log::Warning) << "[Warning] Cross path may be wrong!" << endl;
        ifs.open(env.cross_path);
        if (!ifs.is_open()) {
            Log(Log::Fatal) << "[Fatal] Can't open cross.txt" << endl;
            valid = false;
        } else {
            string line;
            getline(ifs, line);
            crosses.reserve(100);
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
                crosses.push_back(move(Cross(
					changeToZeroID(id, CrossMap),
                    north == -1 ? -1 : changeToZeroID(north, RoadMap),
                    east == -1 ? -1 : changeToZeroID(east, RoadMap),
                    south == -1 ? -1 : changeToZeroID(south, RoadMap),
                    west == -1 ? -1 : changeToZeroID(west, RoadMap)
                )));
            }
        }
        ifs.close();
    }
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
