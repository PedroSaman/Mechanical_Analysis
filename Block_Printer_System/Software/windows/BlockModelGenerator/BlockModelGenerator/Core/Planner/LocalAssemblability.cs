using System;
using System.Collections.Generic;
using System.Linq;
using Core.Geometry;

namespace Core.Planner
{
    enum LocalAssemblabilityTag
    {
        Conflicts,
        Isolated,
        CausesRotation,
        SupportedByNeightbor,
        PartiallySupported,
        Supported,
        HasStableBase
    }
    static class LocalAssemblabilityTagExtension
    {
        private static readonly LocalAssemblabilityTag[] assemblableTags = new[]{
            LocalAssemblabilityTag.HasStableBase,
            LocalAssemblabilityTag.Supported,
            LocalAssemblabilityTag.PartiallySupported,
            LocalAssemblabilityTag.SupportedByNeightbor
            };
        public static bool IsAssemblable(this LocalAssemblabilityTag tag) => assemblableTags.Contains(tag);
    }
    class LocalAssemblability : IAssemblability
    {
        public readonly LocalAssemblabilityTag Tag;
        public readonly int ConnectedLowerBlockCount;
        public readonly int ConnectedLowerConvexCount;
        public readonly Grid3 BlockSize;
        public bool IsAssemblable => this.Tag.IsAssemblable();
        public LocalAssemblability(LocalAssemblabilityTag tag,
            int connectedLowerBlockCount,
            int connectedLowerConvexCount,
            Grid3 blockSize)
        {
            this.Tag = tag;
            this.ConnectedLowerBlockCount = connectedLowerBlockCount;
            this.ConnectedLowerConvexCount = connectedLowerConvexCount;
            this.BlockSize = blockSize;
        }
        public int CompareTo(LocalAssemblability other)
        {
            //ブロック同士の位置関係から求めた組立可能性による比較
            var studAssemblabilityComparison = this.Tag.CompareTo(other.Tag);
            if (studAssemblabilityComparison != 0) return studAssemblabilityComparison;
            //接続ブロック数による比較
            var connectedBlockComparison = this.ConnectedLowerBlockCount.CompareTo(other.ConnectedLowerBlockCount);
            if (connectedBlockComparison != 0) return connectedBlockComparison;
            //接続凸部数による比較
            var connectedConvexComparison = this.ConnectedLowerConvexCount.CompareTo(other.ConnectedLowerConvexCount);
            if (connectedConvexComparison != 0) return connectedConvexComparison;
            //ブロックサイズによる比較
            int evaluateBlockSize(Grid3 size) => size.X + size.Y + size.Z;
            return evaluateBlockSize(this.BlockSize).CompareTo(evaluateBlockSize(other.BlockSize));
        }
        public int CompareTo(IAssemblability other) => other is LocalAssemblability a ? this.CompareTo(a) : throw new ArgumentException($"the type of {nameof(other)} must be {nameof(LocalAssemblability)}");
        public override int GetHashCode() => this.Tag.GetHashCode() ^ this.ConnectedLowerBlockCount.GetHashCode() ^ this.ConnectedLowerConvexCount.GetHashCode() ^ this.BlockSize.GetHashCode();
    }
}
