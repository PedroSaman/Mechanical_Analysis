using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Core.Geometry;
using Core.Attribute;

namespace Core
{
    class AssemblyComponent : IEquatable<AssemblyComponent>
    {
        public readonly Block Block;
        public readonly Grid3 Position;
        public readonly GenericCollection<IAssemblyAttribute> AssemblyAttributeCollection;
        public readonly IReadOnlyCollection<Grid3> OccupyingPositions;
        public Grid3 Size => this.Block.Size;
        public GenericCollection<IBlockAttribute> BlockAttributeCollection => this.Block.BlockAttributeCollection;
        public int RightPosition => this.Position.X + this.Size.X - 1;
        public int LeftPosition => this.Position.X;
        public int FrontPosition => this.Position.Y + this.Size.Y - 1;
        public int BackPosition => this.Position.Y;
        public int TopPosition => this.Position.Z + this.Size.Z - 1;
        public int BottomPosition => this.Position.Z;
        public IEnumerable<Grid3> RightPositions => this.OccupyingPositions.MaxElements(p => p.X);
        public IEnumerable<Grid3> LeftPositions => this.OccupyingPositions.MinElements(p => p.X);
        public IEnumerable<Grid3> FrontPositions => this.OccupyingPositions.MaxElements(p => p.Y);
        public IEnumerable<Grid3> BackPositions => this.OccupyingPositions.MinElements(p => p.Y);
        public IEnumerable<Grid3> TopPositions => this.OccupyingPositions.MaxElements(p => p.Z);
        public IEnumerable<Grid3> BottomPositions => this.OccupyingPositions.MinElements(p => p.Z);
        public AssemblyComponent(Grid3 size, Grid3 position) : this(new Block(size), position, GenericCollection.Empty<IAssemblyAttribute>()) { }
        public AssemblyComponent(Block block, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributeCollection)
        {
            this.Block = block;
            this.Position = position;
            this.AssemblyAttributeCollection = assemblyAttributeCollection;
            this.OccupyingPositions =
                (from width in Enumerable.Range(0, this.Size.X)
                 from height in Enumerable.Range(0, this.Size.Y)
                 from depth in Enumerable.Range(0, this.Size.Z)
                 select this.Position + new Grid3(width, height, depth)).ToArray();
        }
        public AssemblyComponent(Grid3 size, GenericCollection<IBlockAttribute> blockAttributeCollection, Grid3 position, GenericCollection<IAssemblyAttribute> assemblyAttributeCollection)
        : this(new Block(size, blockAttributeCollection), position, assemblyAttributeCollection) { }
        public bool Equals(AssemblyComponent other)
        {
            return this.Block.Equals(other.Block)
            && this.Position.Equals(other.Position)
            && this.AssemblyAttributeCollection.Equals(other.AssemblyAttributeCollection);
        }
        public override int GetHashCode() => this.Block.GetHashCode() ^ this.Position.GetHashCode() ^ this.AssemblyAttributeCollection.GetHashCode();
    }
    static class AssemblyComponentExtension
    {
        public static bool Occupy(this AssemblyComponent component, Grid3 grid) => component.OccupyingPositions.Contains(grid);
        public static bool Touches(this AssemblyComponent component, AssemblyComponent other) =>
        (from p1 in component.OccupyingPositions
         from p2 in other.OccupyingPositions
         let d = Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y) + Math.Abs(p1.Z - p2.Z)
         where d <= 1
         select d).Any();
    }
}