using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Core.Geometry;
using Core.Attribute;

namespace Core.Planner
{
    class AssemblyPlan : IEnumerable<Assembly>
    {
        private readonly IReadOnlyCollection<Assembly> assemblies;
        public AssemblyPlan() => this.assemblies = Enumerable.Empty<Assembly>().ToList();
        public AssemblyPlan(IEnumerable<Assembly> assemblies) => this.assemblies = assemblies.ToList();
        public AssemblyPlan AddAssembly(Assembly assembly) => new AssemblyPlan(this.Append(assembly));
        public AssemblyPlan ChangeLastAssembly(Assembly assembly) => new AssemblyPlan(this.Take(this.Count() - 1).Append(assembly));
        public AssemblyPlan TrimEmptyAssembly() => new AssemblyPlan(this.Where(assembly => assembly.Any()));
        public IEnumerator<Assembly> GetEnumerator() => this.assemblies.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => this.assemblies.GetEnumerator();
    }
    static class AssemblyPlanExtension
    {
        public static AssemblyPlan ArrangePlan(this AssemblyPlan plan, Grid2 areaSize, Grid3 areaOrigin)
        {
            var arrangedAssemblies = new List<Assembly>();
            var assemblyArea = AssemblyArea.Default;
            var assemblyOrigin = areaOrigin;
            foreach (var assembly in plan)
            {
                var sizeX = assembly.RightPosition - assembly.LeftPosition + 1;
                var sizeY = assembly.FrontPosition - assembly.BackPosition + 1;
                if (assemblyOrigin.X + sizeX > areaSize.X)
                {
                    assemblyOrigin = new Grid3(areaOrigin.X, assemblyOrigin.Y + sizeY + 1, assemblyOrigin.Z);
                    if (assemblyOrigin.Y + sizeY > areaSize.Y)
                    {
                        assemblyArea = assemblyArea.Next();
                        assemblyOrigin = areaOrigin;
                    }
                }
                var previousAssemblOrigin = new Grid3(assembly.LeftPosition, assembly.BackPosition, assembly.BottomPosition);
                var transpose = assemblyOrigin - previousAssemblOrigin;
                var arrangedComponents =
                    from component in assembly
                    let position = component.Position + transpose
                    let assemblyAttributeCollection = component.AssemblyAttributeCollection.Append(assemblyArea)
                    select new AssemblyComponent(component.Size, component.BlockAttributeCollection, position, assemblyAttributeCollection);
                arrangedAssemblies.Add(new Assembly(arrangedComponents));
                assemblyOrigin += new Grid3(sizeX + 1, 0, 0);
            }
            return new AssemblyPlan(arrangedAssemblies);
        }
    }
}
