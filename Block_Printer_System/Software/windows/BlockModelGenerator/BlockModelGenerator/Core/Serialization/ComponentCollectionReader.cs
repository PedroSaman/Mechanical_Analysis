using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using System.Linq;
using Core.Geometry;
using Core.Attribute;

namespace Core.Serialization
{
    class ComponentCollectionReader : IComponentCollectionReader
    {
        private static readonly Regex lineRegex = new Regex(@"^(?<x>\d+) (?<y>\d+) (?<z>\d+) (?<sizeX>\d+) (?<sizeY>\d+) (?<sizeZ>\d+) (?<color>\d+)$");
        public ComponentCollectionReader() { }
        public IEnumerable<AssemblyComponent> ReadComponentsFrom(string filename) => ReadComponents(filename);
        private static IEnumerable<AssemblyComponent> ReadComponents(string filename)
        {
            var lines = File.ReadAllLines(filename);
            foreach (var line in lines)
            {
                var match = lineRegex.Match(line);
                if (!match.Success)
                {
                    Console.WriteLine($"'{line}'はブロック定義の正規表現にマッチしませんでした");
                    continue;
                }
                var x = int.Parse(match.Groups["x"].Value);
                var y = int.Parse(match.Groups["y"].Value);
                var z = int.Parse(match.Groups["z"].Value);
                var position = new Grid3(x, y, z);
                var sizeX = int.Parse(match.Groups["sizeX"].Value);
                var sizeY = int.Parse(match.Groups["sizeY"].Value);
                var sizeZ = int.Parse(match.Groups["sizeZ"].Value);
                var size = new Grid3(sizeX, sizeY, sizeZ);
                var color = new Color(int.Parse(match.Groups["color"].Value));
                var blockAttributeCollection = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { color });
                var assemblyAttributeCollection = GenericCollection.Empty<IAssemblyAttribute>();
                yield return new AssemblyComponent(size, blockAttributeCollection, position, assemblyAttributeCollection);
            }
        }
    }
}
