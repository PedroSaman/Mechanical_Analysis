using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace Core.Geometry
{
    [DataContract]
    readonly struct Grid2 : IEquatable<Grid2>
    {
        public static readonly Grid2 Origin = new Grid2(0, 0);
        public static readonly Grid2 XUnit = new Grid2(1, 0);
        public static readonly Grid2 YUnit = new Grid2(0, 1);
        [DataMember]
        public readonly int X;
        [DataMember]
        public readonly int Y;
        public Grid2(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }
        public bool Equals(Grid2 other) => this.X == other.X && this.Y == other.Y;
        public override int GetHashCode() => this.X.GetHashCode() ^ this.Y.GetHashCode();
        public static Grid2 operator -(Grid2 grid) => new Grid2(-grid.X, -grid.Y);
        public static Grid2 operator +(Grid2 left, Grid2 right) => new Grid2(left.X + right.X, left.Y + right.Y);
        public static Grid2 operator -(Grid2 left, Grid2 right) => new Grid2(left.X - right.X, left.Y - right.Y);
        public static Grid2 operator *(int left, Grid2 right) => new Grid2(left * right.X, left * right.Y);
    }
}