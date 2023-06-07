#include <vfh_local_planner/polar_histogram.h>

namespace vfh_local_planner{
    class VFHPlanner {
    private:
        Polar_histogram polar_histogram_;
    public:
        VFHPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util, unsigned int number_sector): polar_histogram_(number_sector){}
    };
    
}