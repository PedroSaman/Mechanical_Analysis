using System;
using System.Collections.Generic;
using System.Runtime.Serialization;

namespace Core.Geometry
{
    [DataContract]
    readonly struct Grid3 : IEquatable<Grid3>
    {
        public static readonly Grid3 Origin = new Grid3(0, 0, 0);
        public static readonly Grid3 Unit = new Grid3(1, 1, 1);
        public static readonly Grid3 XUnit = new Grid3(1, 0, 0);
        public static readonly Grid3 YUnit = new Grid3(0, 1, 0);
        public static readonly Grid3 ZUnit = new Grid3(0, 0, 1);
        [DataMember]
        public readonly int X;
        [DataMember]
        public readonly int Y;
        [DataMember]
        public readonly int Z;
        public Grid3(int x, int y, int z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }
        public bool Equals(Grid3 other) => this.X == other.X && this.Y == other.Y && this.Z == other.Z;
        public override int GetHashCode() => this.X.GetHashCode() ^ this.Y.GetHashCode() ^ this.Z.GetHashCode();
        public static Grid3 operator -(Grid3 point) => new Grid3(-point.X, -point.Y, -point.Z);
        public static Grid3 operator +(Grid3 left, Grid3 right) => new Grid3(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        public static Grid3 operator -(Grid3 left, Grid3 right) => new Grid3(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
    }
}