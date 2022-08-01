using System;

namespace Core.Planner
{
    interface IAssemblability : IComparable<IAssemblability>
    {
        bool IsAssemblable { get; }
    }
}
