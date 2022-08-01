namespace Core.Attribute
{
    /// <summary>
    /// このブロックがサポートブロックであることを表す．
    /// </summary>
    readonly struct Support : IBlockAttribute
    {
        public static readonly Support Instance = new Support();
        public bool Equals(IBlockAttribute other) => other is Support;
        public override int GetHashCode() => 0;
    }
}
