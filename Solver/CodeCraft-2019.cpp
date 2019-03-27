#include <iostream>

#include "Solver.h"

#ifdef _WIN32
#include "Visualization.h"
#endif // _WIN32


using namespace std;
using namespace codecraft2019;

int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;
	
	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);
	
	std::cout << "carPath is " << carPath.data() << std::endl;
	std::cout << "roadPath is " << roadPath.data() << std::endl;
	std::cout << "crossPath is " << crossPath.data() << std::endl;
	std::cout << "answerPath is " << answerPath.data() << std::endl;

    Environment env(argv[1], argv[2], argv[3], argv[4]);
    Instance ins(env);
    Output output(&ins);
    Configure cfg;
	
    Solver solver(&ins, &output, &env, &cfg);
    solver.run();
	//solver.init_solution_once();
	//solver.check_solution();
    output.save(env);

#ifdef _WIN32
    Visualization vis(&ins, &solver.timeslice);
    string zjl_laptop_path = "E:\\2019\\CodeCraft\\CodeCraft2019\\Deploy\\Visualization\\config_1.html";
    string zjl_smart_path = "E:\\CodeCraft\\CodeCraft2019\\Deploy\\Visualization\\config_12.html";
    vis.draw(zjl_smart_path);
#endif // _WIN32

	return 0;
}