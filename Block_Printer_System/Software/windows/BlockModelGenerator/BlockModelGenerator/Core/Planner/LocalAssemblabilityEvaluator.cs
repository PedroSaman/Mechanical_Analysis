using System;
using System.Collections.Generic;
using System.Linq;
using Core;
using Core.Geometry;

namespace Core.Planner
{
    class LocalAssemblabilityEvaluator : IAssemblabilityEvaluator
    {
        public IAssemblability GetAssemblability(Assembly assembly, AssemblyComponent component)
        {
            return assembly.GetPartialAssemblability(component);
        }
    }
    static class AssemblyExtensionForLocalAssemblabilityEvaluator
    {
        /// <summary>
        /// 指定したブロックの上面と接続しているブロックを返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static IEnumerable<AssemblyComponent> GetConnectedUpperComponentsTo(this Assembly assembly, AssemblyComponent component)
        {
            var upperPositions = component.TopPositions.Select(p => p + Grid3.ZUnit);
            var upperLayerAssemblyComponents = assembly.GetComponentsByLowerPosition(component.TopPosition + 1);
            return upperLayerAssemblyComponents.Where(b => upperPositions.Any(p => b.Occupy(p)));
        }
        /// <summary>
        /// 指定したブロックの底面と接続しているブロックを返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static IEnumerable<AssemblyComponent> GetConnectedLowerComponentsTo(this Assembly assembly, AssemblyComponent component)
        {
            var lowerPositions = component.BottomPositions.Select(p => p - Grid3.ZUnit);
            var lowerLayerAssemblyComponents = assembly.GetComponentsByUpperPosition(component.BottomPosition - 1);
            return lowerLayerAssemblyComponents.Where(b => lowerPositions.Any(p => b.Occupy(p)));
        }
        /// <summary>
        /// 指定したブロックの底面のうち，他のブロックと接続する座標を返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static IEnumerable<Grid3> GetConnectedLowerConvexesTo(this Assembly assembly, AssemblyComponent component) => component.BottomPositions
            .Select(p => p - Grid3.ZUnit)
            .Where(p => assembly.Occupy(p));
        public static IEnumerable<(ConvexHull upper, ConvexHull lower)> GetSupportingConvexHullPairs(this Assembly assembly, AssemblyComponent component)
        {
            var connectedAssemblyComponents = assembly.GetConnectedLowerComponentsTo(component);
            IEnumerable<Grid3> getConnectedConvexes(AssemblyComponent upper, AssemblyComponent lower) => lower.TopPositions
            .Select(p => p + Grid3.ZUnit)
            .Where(p => upper.Occupy(p));
            ConvexHull ToConvexHull(IEnumerable<Grid3> grids)
            {
                IEnumerable<Grid2> getPlanarCornerVertexes(Grid3 grid)
                {
                    return
                    from x in EnumerableExtension.FromTo(-1, 1)
                    from y in EnumerableExtension.FromTo(-1, 1)
                    let planar = new Grid2(grid.X, grid.Y)
                    let center = 3 * planar
                    let offset = new Grid2(x, y)
                    select center + offset;
                }
                var points = grids
                    .SelectMany(p => getPlanarCornerVertexes(p))
                    .Distinct();
                return new ConvexHull(points);
            }
            foreach (var connectedAssemblyComponent in connectedAssemblyComponents)
            {
                var upperConvexes = getConnectedConvexes(component, connectedAssemblyComponent);
                var lowerConvexes = connectedAssemblyComponents.SelectMany(lowerAssemblyComponent => assembly.GetConnectedLowerConvexesTo(lowerAssemblyComponent));
                var upperConvexHull = ToConvexHull(upperConvexes);
                var lowerConvexHull = ToConvexHull(lowerConvexes);
                yield return (upperConvexHull, lowerConvexHull);
            }
        }
        /// <summary>
        /// 指定したブロックが部品や底面と衝突するか返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static bool ConflictWith(this Assembly assembly, AssemblyComponent component)
        {
            //ブロックが部品の最下段よりも低い位置にある場合は衝突すると判定する
            if (assembly.Any())
            {
                if (component.BottomPosition < assembly.BottomPosition) return true;
            }
            //
            return component.OccupyingPositions.Any(p => assembly.Occupy(p));
        }
        /// <summary>
        /// 指定したブロックが部品と孤立しており，組み立てパッドの直上にない場合にtrueを返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static bool FloatAndIsolates(this Assembly assembly, AssemblyComponent component) =>
            component.Position.Z > assembly.BottomPosition
            && !GetConnectedLowerComponentsTo(assembly, component).Any();
        /// <summary>
        /// 指定したブロックを組み立てるための安定土台を有するか返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static bool HasStableBaseFor(this Assembly assembly, AssemblyComponent component)
        {
            //最初に組み立てるブロックなら必ず安定状態とする
            if (!assembly.Any()) return true;
            //最下段は安定
            if (component.BottomPosition == assembly.BottomPosition) return true;
            //下から2段目は，下段ブロックと接続をもつ場合のみ安定
            if (component.BottomPosition == assembly.BottomPosition + 1)
                return assembly.GetConnectedLowerComponentsTo(component).Any();
            //ブロックの下の座標を列挙する
            var basePositions =
                from z in EnumerableExtension.FromTo(assembly.BottomPosition, component.Position.Z - 1)
                let possession = component.BottomPositions
                let checkPositions = possession.Select(p => new Grid3(p.X, p.Y, z))
                select checkPositions;
            //列挙した座標すべてが他のブロックによって占有されていれば安定土台と判定する
            return basePositions
                .SelectMany(positions => positions)
                .All(p => assembly.Occupy(p));
        }
        /// <summary>
        /// 指定したブロックのとなりに，そのブロックと隣接していて，かつそのブロックと結合下段ブロックを共有しているようなブロックが存在するか返す．
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="component"></param>
        /// <returns></returns>
        public static bool SupportByNeighborComponent(this Assembly assembly, AssemblyComponent component)
        {
            var lowerAssemblyComponents = GetConnectedLowerComponentsTo(assembly, component);
            //指定したブロックと土台を共有しているブロックを求める
            var assembledUpperAssemblyComponents = lowerAssemblyComponents
               .SelectMany(b => GetConnectedUpperComponentsTo(assembly, b));
            //土台を共有しており，かつ隣接しているブロックがあり，そのブロックが複数の下段ブロックと接続しているか判定
            return assembledUpperAssemblyComponents
                .Where(b => component.Touches(b))
                .Any(b => assembly.GetConnectedLowerComponentsTo(b).Count() >= 2);
        }
        public static LocalAssemblability GetPartialAssemblability(this Assembly assembly, AssemblyComponent component)
        {
            //凸包を形成しなくてもわかる組立可能性について先に調べる
            LocalAssemblabilityTag? nullableTag =
            assembly.ConflictWith(component) ? (LocalAssemblabilityTag?)LocalAssemblabilityTag.Conflicts
            : assembly.HasStableBaseFor(component) ? (LocalAssemblabilityTag?)LocalAssemblabilityTag.HasStableBase
            : assembly.FloatAndIsolates(component) ? (LocalAssemblabilityTag?)LocalAssemblabilityTag.Isolated
            : assembly.SupportByNeighborComponent(component) ? (LocalAssemblabilityTag?)LocalAssemblabilityTag.SupportedByNeightbor
            : null;
            //組立可能性が決定しなかった場合は凸包を作成し，その凸包を使って組立可能性を決定する
            if (!nullableTag.HasValue)
            {
                var convexHullPairs = assembly.GetSupportingConvexHullPairs(component);
                nullableTag = GetLocalAssemblabilityTag(convexHullPairs);
            }
            //
            var connectedAssemblyComponentCount = assembly.GetConnectedLowerComponentsTo(component).Count();
            var connectedConvexCount = assembly.GetConnectedLowerConvexesTo(component).Count();
            return new LocalAssemblability(nullableTag.Value, connectedAssemblyComponentCount, connectedConvexCount, component.Size);
        }
        private static LocalAssemblabilityTag GetLocalAssemblabilityTag(IEnumerable<(ConvexHull upper, ConvexHull lower)> convexHullPairs)
        {
            var tags = new List<LocalAssemblabilityTag>();
            foreach (var (upper, lower) in convexHullPairs)
            {
                var tag =
                lower.Contains(upper) ? LocalAssemblabilityTag.Supported
                : lower.HasInnerCommonPoint(upper) ? LocalAssemblabilityTag.PartiallySupported
                : LocalAssemblabilityTag.CausesRotation;
                //組立不可能なパターンが見つかった場合はすぐに返す
                if (!tag.IsAssemblable()) return tag;
                tags.Add(tag);
            }
            return tags.Min();
        }
    }
}