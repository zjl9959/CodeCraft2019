#include <iostream>

#include "Solver.h"
#include "Visualization.h"

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
    //Visualization vis(&ins, &solver.timeslice);
   // vis.draw("F:\\ÕÒ¹¤×÷\\codecraft2019\\CodeCraft2019\\Deploy\\Visualization\\config_0.html");

	return 0;
}