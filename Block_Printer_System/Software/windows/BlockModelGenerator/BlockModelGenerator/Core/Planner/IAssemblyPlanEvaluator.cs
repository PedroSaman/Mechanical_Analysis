using System;
using System.Collections.Generic;

namespace Core.Planner
{
    interface IAssemblyPlanEvaluator
    {
        AssemblyPlan SelectBestPlan(IEnumerable<AssemblyPlan> plans);
    }
}
