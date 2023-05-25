using System;
using System.Collections.Generic;
using System.Linq;
using Core.Geometry;
using Core.Planner;

namespace Core.Blockalization
{
    static class VoxelConverter
    {
        public static Assembly CreateBlockModel(IEnumerable<Grid3> voxels)
        {
            Assembly assembly = new Assembly();
            var layers = voxels.GroupBy(v => v.Z).OrderBy(group => group.Key);
            //段ごとにブロックを組み立てる
            foreach (var layer in layers)
            {
                AssembleLayer(assembly, layer.ToArray());
            }
            Console.WriteLine($"ブロック{assembly.Count()}個のモデルへ変換しました");
            Console.WriteLine($"孤立ブロックを削除します");
            var slimedAssembly = RemoveIsolatedComponents(assembly);
            Console.WriteLine($"ブロック{slimedAssembly.Count()}個のモデルへ変換しました");
            return slimedAssembly;
        }
        private static void AssembleLayer(Assembly assembly, IReadOnlyCollection<Grid3> voxelLayer)
        {
            if (!voxelLayer.Any()) return;

            //var enableOffset = voxelLayer.First().Z % 2 == 0;
            var change = voxelLayer.First().Z % 2 == 0;
            var offset = Grid2.Origin;
            
            if(change)
            {
                //8x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(8, 2), offset);
                //余ったボクセルで，8x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(8, 2));

                //8x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 8), offset);
                //余ったボクセルで，8x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 8));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(4, 2), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(4, 2));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 4), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 4));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(3, 2), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(3, 2));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 3), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 3));

                //4x1ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(4, 1), offset);
                //余ったボクセルで，4x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(4, 1));

                //4x1ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(1, 4), offset);
                //余ったボクセルで，4x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 4));

                //2x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 2), offset);
                //余ったボクセルで，2x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 2));

                //1x3ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(3, 1), offset);
                //余ったボクセルで，3x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(3, 1));
                
                //1x3ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(1, 3), offset);
                //余ったボクセルで，3x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 3));

                //1x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 1), offset);
                //余ったボクセルで，2x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 1));

                //1x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(1, 2), offset);
                //余ったボクセルで，2x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 2));

                //1x1ブロックを組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 1));
            }
            else
            {
                //8x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 8), offset);
                //余ったボクセルで，8x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 8));

                //8x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(8, 2), offset);
                //余ったボクセルで，8x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(8, 2));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 4), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 4));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(4, 2), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(4, 2));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 3), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 3));

                //4x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(3, 2), offset);
                //余ったボクセルで，4x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(3, 2));

                //4x1ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(1, 4), offset);
                //余ったボクセルで，4x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 4));

                //4x1ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(4, 1), offset);
                //余ったボクセルで，4x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(4, 1));

                //2x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 2), offset);
                //余ったボクセルで，2x2ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 2));

                //1x3ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(1, 3), offset);
                //余ったボクセルで，3x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 3));

                //1x3ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(3, 1), offset);
                //余ったボクセルで，3x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(3, 1));

                //1x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(1, 2), offset);
                //余ったボクセルで，2x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 2));

                //1x2ブロックを互い違いに組立
                ArrangeComponents(assembly, voxelLayer, new Grid2(2, 1), offset);
                //余ったボクセルで，2x1ブロックにできるとこがあれば組立
                FillComponents(assembly, voxelLayer, new Grid2(2, 1));

                //1x1ブロックを組立
                FillComponents(assembly, voxelLayer, new Grid2(1, 1));
            }

        }
        private static void ArrangeComponents(Assembly assembly, IReadOnlyCollection<Grid3> voxelLayer, Grid2 planarBlockSize, Grid2 planarOffset)
        {
            if (!voxelLayer.Any()) return;
            var z = voxelLayer.First().Z;
            if (voxelLayer.Any(v => v.Z != z)) throw new ArgumentException("voxels have dirrefent Z element");
            
            var xmin = voxelLayer.Min(v => v.X);
            var ymin = voxelLayer.Min(v => v.Y);
            var xmax = voxelLayer.Max(v => v.X) - planarBlockSize.X + 1;
            var ymax = voxelLayer.Max(v => v.Y) - planarBlockSize.Y + 1;
            if (xmax < xmin || ymax < ymin)
            {
                return;
            }
            var positionCandidates =
                from x in EnumerableExtension.FromTo(xmin, xmax)
                from y in EnumerableExtension.FromTo(ymin, ymax)
                where x % planarBlockSize.X == 0
                where y % planarBlockSize.Y == 0
                let p = new Grid3(x, y, z)
                let offset = new Grid3(planarOffset.X, planarOffset.Y, 0)
                let shifted = p + offset
                select shifted;
            var blockSize = new Grid3(planarBlockSize.X, planarBlockSize.Y, 1);
            ArrangeComponentsWithPositionCandidates(assembly, voxelLayer, positionCandidates, blockSize);
        }
        private static void FillComponents(Assembly assembly, IReadOnlyCollection<Grid3> voxelLayer, Grid2 planarBlockSize)
        {
            if (!voxelLayer.Any()) return;
            var z = voxelLayer.First().Z;
            if (voxelLayer.Any(v => v.Z != z)) throw new ArgumentException("voxels have dirrefent Z element");
            //
            var xmin = voxelLayer.Min(v => v.X);
            var ymin = voxelLayer.Min(v => v.Y);
            var xmax = voxelLayer.Max(v => v.X) - planarBlockSize.X + 1;
            var ymax = voxelLayer.Max(v => v.Y) - planarBlockSize.Y + 1;
            if (xmax < xmin || ymax < ymin)
            {
                return;
            }
            var positionCandidates =
                from x in EnumerableExtension.FromTo(xmin, xmax)
                from y in EnumerableExtension.FromTo(ymin, ymax)
                select new Grid3(x, y, z);
            var blockSize = new Grid3(planarBlockSize.X, planarBlockSize.Y, 1);
            ArrangeComponentsWithPositionCandidates(assembly, voxelLayer, positionCandidates, blockSize);
        }
        private static void ArrangeComponentsWithPositionCandidates(Assembly assembly, IReadOnlyCollection<Grid3> voxelLayer, IEnumerable<Grid3> positionCandidates, Grid3 blockSize)
        {
            foreach (var positionCandidate in positionCandidates)
            {
                var component = new AssemblyComponent(blockSize, positionCandidate);
                //Console.WriteLine($"component position ({component.Position.X},{component.Position.Y},{component.Position.Z})");
                if (assembly.ConflictWith(component))
                {
                    continue;
                }
                if (component.OccupyingPositions.Any(p => !voxelLayer.Contains(p)))
                {
                    continue;
                }
                assembly.AddComponent(component);
            }
        }
        private static Assembly RemoveIsolatedComponents(Assembly assembly)
        {
            var nextConnectedcomponents =
                (from component in assembly.AsQueryable()
                 let lowerConnection = assembly.GetConnectedLowerComponentsTo(component)
                 let upperConnection = assembly.GetConnectedUpperComponentsTo(component)
                 where lowerConnection.Any() || upperConnection.Any()
                 select component).ToArray();
            //全ブロックが他のブロックと接続をもっていれば終了
            if (assembly.Count() == nextConnectedcomponents.Length) return assembly;
            //孤立ブロックを取り除いて，もう一度検査
            return RemoveIsolatedComponents(new Assembly(nextConnectedcomponents));
        }
    }
}
