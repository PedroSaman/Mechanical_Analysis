using System.Collections.Generic;

namespace Core.Planner
{
    interface IAssemblyPlanner
    {
        AssemblyPlan GeneratePlan(IEnumerable<AssemblyComponent> components);
    }
}
