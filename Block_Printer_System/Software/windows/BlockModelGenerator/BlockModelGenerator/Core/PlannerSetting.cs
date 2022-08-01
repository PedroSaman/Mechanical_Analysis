using System.Collections.Generic;
using System.Runtime.Serialization;
using Core.Geometry;

namespace Core
{
    [DataContract]
    class PlannerSetting
    {
        public static readonly PlannerSetting Default = new PlannerSetting()
        {
            MaxSandwichedSupportCount = 20,
            MaxSubassemblyCount = 4,
            MinSubassemblyLayerCount = 2,
            AssemblyAreaSize = new Grid2(20, 20),
            AssemblyAreaOrigin = new Grid3(0, 0, 1),
            AvailableSupportBlockSizes = new[] { new Grid3(1, 1, 1) },
        };
        [DataMember] public int MaxSandwichedSupportCount { get; private set; }
        [DataMember] public int MaxSubassemblyCount { get; private set; }
        [DataMember] public int MinSubassemblyLayerCount { get; private set; }
        [DataMember] public Grid2 AssemblyAreaSize { get; private set; }
        [DataMember] public Grid3 AssemblyAreaOrigin { get; private set; }
        [DataMember] public IEnumerable<Grid3> AvailableSupportBlockSizes { get; private set; }
        private PlannerSetting() { }
    }
}
