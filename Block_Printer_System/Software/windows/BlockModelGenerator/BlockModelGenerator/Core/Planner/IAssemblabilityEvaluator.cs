namespace Core.Planner
{
    interface IAssemblabilityEvaluator
    {
        IAssemblability GetAssemblability(Assembly assembly, AssemblyComponent component);
    }
}
