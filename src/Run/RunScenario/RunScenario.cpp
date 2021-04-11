#include "RunScenario.hpp"

#include <vector>

#include "ScenarioLoader.h"
#include "stats.hpp"
#include "../GetPath/GetPath.hpp"
#include "../../Utility/Timer.h"

void RunScenario(const std::string &scen_filename, const PreprocessingData &preprocessing_data)
{
  const int width = preprocessing_data.get_graph().get_width();
  const int height = preprocessing_data.get_graph().get_height();

  std::vector<xyLoc> thePath;

  ScenarioLoader scen(scen_filename.c_str());

  Timer t;
  std::vector<stats> experimentStats;
  for (int x = 0; x < scen.GetNumExperiments(); x++)
    {
    //printf("%d of %d\n", x+1, scen.GetNumExperiments());
    thePath.resize(0);
    experimentStats.resize(x+1);
    bool done;
    do {
      xyLoc s, g;
      s.x = scen.GetNthExperiment(x).GetStartX();
      s.y = scen.GetNthExperiment(x).GetStartY();
      g.x = scen.GetNthExperiment(x).GetGoalX();
      g.y = scen.GetNthExperiment(x).GetGoalY();

      t.StartTimer();
      done = GetPath(preprocessing_data, s + xyLoc(1,1), g + xyLoc(1,1), thePath);
      t.EndTimer();

      experimentStats[x].times.push_back(t.GetElapsedTime());
      experimentStats[x].lengths.push_back(thePath.size());
      for (unsigned int t = experimentStats[x].path.size(); t < thePath.size(); t++)
        experimentStats[x].path.push_back(thePath[t]);
    } while (done == false);

    }

  for (unsigned int x = 0; x < experimentStats.size(); x++)
  {
    printf("%s\ttotal-time\t%f\tmax-time-step\t%f\ttime-20-moves\t%f\ttotal-len\t%f\tsubopt\t%f\t", scen_filename.c_str(),
         experimentStats[x].GetTotalTime(), experimentStats[x].GetMaxTimestep(), experimentStats[x].Get20MoveTime(),
         experimentStats[x].GetPathLength(), experimentStats[x].GetPathLength()/scen.GetNthExperiment(x).GetDistance());
    if (scen.GetNthExperiment(x).GetStartX() != experimentStats[x].path[0].x - 1 ||
        scen.GetNthExperiment(x).GetStartY() != experimentStats[x].path[0].y - 1 ||
        scen.GetNthExperiment(x).GetGoalX() != experimentStats[x].path.back().x - 1 ||
        scen.GetNthExperiment(x).GetGoalY() != experimentStats[x].path.back().y - 1)
    {
      printf("path does not match experiment\n");
      std::cout << scen.GetNthExperiment(x).GetStartX() << " vs " << experimentStats[x].path[0].x << std::endl;
      std::cout << scen.GetNthExperiment(x).GetStartY() << " vs " << experimentStats[x].path[0].y << std::endl;
      std::cout << scen.GetNthExperiment(x).GetGoalX() << " vs " << experimentStats[x].path.back().x << std::endl;
      std::cout << scen.GetNthExperiment(x).GetGoalY() << " vs " << experimentStats[x].path.back().y << std::endl;
    }
    else if (experimentStats[x].path.size() == 0 || experimentStats[x].ValidatePath(width, height, preprocessing_data.get_graph()))
    {
      printf("valid\n");
    }
    else {
      printf("invalid\n");
    }
  }
}
