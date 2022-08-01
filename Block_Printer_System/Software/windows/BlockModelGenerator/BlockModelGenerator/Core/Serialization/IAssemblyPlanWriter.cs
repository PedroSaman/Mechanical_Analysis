using Core.Planner;

namespace Core.Serialization
{
    interface IAssemblyPlanWriter
    {
        void WritePlan(string filename, AssemblyPlan plan);
    }
}
