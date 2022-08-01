using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using Core;

namespace Core.Blockalization
{
    static class BlockModelWriter
    {
        public static void WriteBlockModel(string filename, IEnumerable<AssemblyComponent> copmponents)
        {
            var header = "X Y Z sizeX sizeY sizeZ";
            var lines =
                from component in copmponents
                let p = component.Position
                let s = component.Size
                select $"{p.X} {p.Y} {p.Z} {s.X} {s.Y} {s.Z}";
            File.WriteAllLines(filename, lines.Prepend(header).ToArray());
        }
    }
}
