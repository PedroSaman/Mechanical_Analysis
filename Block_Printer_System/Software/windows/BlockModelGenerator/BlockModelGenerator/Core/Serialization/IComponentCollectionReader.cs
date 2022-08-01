using System.Collections.Generic;

namespace Core.Serialization
{
    interface IComponentCollectionReader
    {
        IEnumerable<AssemblyComponent> ReadComponentsFrom(string filename);
    }
}
