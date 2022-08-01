using System;
using System.Text;
using System.Linq;
using System.IO;
using Core.Attribute;
using Core.Planner;

namespace Core.Serialization
{
    class AssemblyPlanWriter : IAssemblyPlanWriter
    {
        public void WritePlan(string filename, AssemblyPlan plan)
        {
            var blocks =
                from block in plan.SelectMany(assembly => assembly)
                let topPosition = block.TopPosition
                orderby topPosition
                select block;
            var lines = blocks.Select(block => ComponentToLine(block))
            .Prepend("#AssemblyArea,X,Y,Z,SizeX,SizeY,SizeZ,ColorIndex,IsSupport,CanPress,ShiftX,ShiftY")
            .ToArray();
            //
            File.WriteAllLines(filename, lines);
        }
        private static string ComponentToLine(AssemblyComponent component)
        {
            StringBuilder builder = new StringBuilder();
            var assemblyArea = component.AssemblyAttributeCollection.Single<AssemblyArea>();
            builder.Append($"{assemblyArea.Index},");
            builder.Append($"{component.Position.X},{component.Position.Y},{component.Position.Z},");
            builder.Append($"{component.Size.X},{component.Size.Y},{component.Size.Z},");
            if (component.BlockAttributeCollection.Exists<Color>())
            {
                var color = component.BlockAttributeCollection.Single<Color>();
                builder.Append($"{color.Index},");
            }
            else
            {
                builder.Append("0,");
            }
            builder.Append(component.BlockAttributeCollection.Exists<Support>() ? "1," : "0,");
            var press = component.AssemblyAttributeCollection.Single<PressCapability>();
            builder.Append(press.Capability ? "1," : "0,");
            var shift = component.AssemblyAttributeCollection.Single<PlaceShift>();
            builder.Append($"{shift.Shift.X},{shift.Shift.Y},");
            return builder.ToString();
        }
    }
}
