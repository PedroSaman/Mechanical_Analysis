using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Core.Geometry
{
    /// <summary>
    /// 格子点の集合で定義される平面内の凸包を表します．
    /// </summary>
    class ConvexHull
    {
        private readonly IReadOnlyCollection<Grid2> componentGrids;

        /// <summary>
        /// 互いに異なる3つ以上の点を用いてこの凸法を初期化する．
        /// </summary>
        /// <param name="grids"></param>
        /// <exception cref="ArgumentException"></exception>
        public ConvexHull(IEnumerable<Grid2> grids)
        {
            var distinction = grids.Distinct().ToArray();
            if (distinction.Length < 3) throw new ArgumentException("too few vertexes was specified.");
            var lineSegments = new List<DirectedSegment>();
            //凸包の最初の始点を決める．この点は凸包の頂点を作る点であることが保証される
            var initialStart = distinction
                .MinElements(p => p.X)
                .MinElements(p => p.Y)
                .First();
            var start = initialStart;
            //ギフト包装法により凸包を構成する有向線分を順に求める．有向線分の向きは凸包を半時計まわりに一周する方向とする．計算量はO(n^2)らしい
            do
            {
                foreach (var end in grids)
                {
                    if (start.Equals(end)) continue;
                    var lineSegment = new DirectedSegment(start, end);
                    //全ての点を左側か直上に持っていける有向線分が見つかれば，その線分を凸包の構成線分とみなす
                    if (grids.All(vertex => lineSegment.SideOf(vertex) != LineSide.Right))
                    {
                        lineSegments.Add(new DirectedSegment(start, end));
                        //現在の終点を次の始点として探索を再開
                        start = end;
                        break;
                    }
                }
                //一周して最初に指定した始点に戻ってきたら終了
            } while (!start.Equals(initialStart));
            //凸包の境界上の点と内部の点を求める
            var gridCandidates =
             from x in EnumerableExtension.FromTo(grids.Min(g => g.X), grids.Max(g => g.X))
             from y in EnumerableExtension.FromTo(grids.Min(g => g.Y), grids.Max(g => g.Y))
             select new Grid2(x, y);
            this.componentGrids =
            (from grid in gridCandidates
             let sides = lineSegments.Select(l => l.SideOf(grid))
             where sides.All(s => s == LineSide.Center || s == LineSide.Left)
             select grid)
            .ToArray();
        }
        /// <summary>
        /// この凸包が指定した点を含んでいるか返す．
        /// </summary>
        /// <param name="grid"></param>
        /// <returns></returns>
        public bool Contains(Grid2 grid) => this.componentGrids.Contains(grid);
        /// <summary>
        /// この凸包が指定した点を内部に含んでいるか返す．内部に含んでいるとは，その点とその点を4方に囲む点全てを含んでいることをいう．
        /// </summary>
        /// <param name="grid"></param>
        /// <returns></returns>
        public bool InnerContains(Grid2 grid)
        {
            var offsets = new[] { Grid2.XUnit, -Grid2.XUnit, Grid2.YUnit, -Grid2.YUnit };
            var surroundingGrids = offsets.Select(o => grid + o);
            return surroundingGrids.Append(grid).All(g => this.Contains(g));
        }
        /// <summary>
        /// この凸包が，指定した凸包を構成する点をすべて含んでいるか返す．
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool Contains(ConvexHull other) => other.componentGrids.All(g => this.Contains(g));
        /// <summary>
        /// この凸包と指定した凸包が少なくとも一つの共通点を持っているか返す．
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool HasCommonPoint(ConvexHull other) => other.componentGrids.Any(g => this.Contains(g));
        /// <summary>
        /// この凸包が，指定した凸包を構成する点のいづれかを内部に含んでいるか返す．
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool HasInnerCommonPoint(ConvexHull other) => other.componentGrids.Any(g => this.InnerContains(g));
    }
}
