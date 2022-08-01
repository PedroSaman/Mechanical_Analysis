using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using Core.Geometry;

namespace Core.Serialization
{
    [DataContract]
    class Voxel
    {
        [DataMember]
        public readonly string id;
        [DataMember]
        public readonly int x;
        [DataMember]
        public readonly int y;
        [DataMember]
        public readonly int z;
    }
    [DataContract]
    class Dimension
    {
        [DataMember]
        public readonly int width;
        [DataMember]
        public readonly int height;
        [DataMember]
        public readonly int depth;
    }
    [DataContract]
    class VoxelModel
    {
        [DataMember]
        public readonly Dimension dimension;
        [DataMember]
        public readonly IEnumerable<Voxel> voxels;
    }
    static class VoxelCollectionReader
    {
        public static IEnumerable<Grid3> LoadVoxelModel(string filename)
        {
            var serializer = new DataContractJsonSerializer(typeof(VoxelModel));
            using (var reader = new StreamReader(filename))
            {
                var model = serializer.ReadObject(reader.BaseStream) as VoxelModel;
                Console.WriteLine($"ボクセルモデル:{filename}を読み込みました．");
                return model.voxels.Select(voxel => new Grid3(voxel.x, voxel.z, voxel.y));
            }
        }
    }
}
