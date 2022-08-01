using System;
using System.Collections.Generic;
using System.Linq;

namespace Core.Geometry
{
    /// <summary>
    /// 有向線分との位置関係を表す．
    /// </summary>
    enum LineSide
    {
        Left,
        Right,
        Center,
    }
    /// <summary>
    /// 平面上の有向線分を表します．
    /// </summary>
    class DirectedSegment
    {
        public readonly Grid2 Start;
        public readonly Grid2 End;
        public Grid2 Vector => this.End - this.Start;
        /// <summary>
        /// 
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <exception cref="ArgumentException"></exception>
        public DirectedSegment(Grid2 start, Grid2 end)
        {
            if (start.Equals(end))
            {
                throw new ArgumentException("start point and end point cannot be the same.");
            }
            this.Start = start;
            this.End = end;
        }
        /// <summary>
        /// 指定した点がこの有向線分のどちら側に位置するか返します．
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public LineSide SideOf(Grid2 point)
        {
            var lineSegmentVector = this.End - this.Start; //この有向線分の始点から終点へのベクトル
            var pointVector = point - this.Start; //始点から指定された点へのベクトル
            //外積によって分類
            var crossProduct = lineSegmentVector.X * pointVector.Y - lineSegmentVector.Y * pointVector.X;
            return crossProduct == 0 ? LineSide.Center
                : crossProduct > 0 ? LineSide.Left
                : LineSide.Right;
        }
    }
}
