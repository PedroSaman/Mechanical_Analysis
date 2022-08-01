using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Core.Geometry;
using Core.Attribute;

namespace Core
{
    /// <summary>
    /// 部品を構成する最小単位である直方体を表す．
    /// </summary>
    class Block : IEquatable<Block>
    {
        public readonly Grid3 Size;
        public readonly GenericCollection<IBlockAttribute> BlockAttributeCollection;
        public Block(Grid3 size) : this(size, GenericCollection.Empty<IBlockAttribute>()) { }
        public Block(Grid3 size, GenericCollection<IBlockAttribute> blockAttributeCollection)
        {
            if (size.X <= 0 || size.Y <= 0 || size.Z <= 0) throw new ArgumentException();
            this.Size = size;
            this.BlockAttributeCollection = blockAttributeCollection;
        }
        public bool Equals(Block other) => this.Size.Equals(other.Size)
            && this.BlockAttributeCollection.Equals(other.BlockAttributeCollection);
        public override int GetHashCode() => this.Size.GetHashCode();
    }
}
