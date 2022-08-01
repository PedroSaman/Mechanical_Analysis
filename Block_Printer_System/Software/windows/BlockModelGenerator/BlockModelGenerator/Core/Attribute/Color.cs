namespace Core.Attribute
{
    /// <summary>
    /// ブロックの色番号を表す．
    /// </summary>
    readonly struct Color : IBlockAttribute
    {
        public static readonly Color Default = new Color(0);
        public readonly int Index;
        public Color(int index) => this.Index = index;
        public bool Equals(IBlockAttribute other) => other is Color color && this.Index == color.Index;
        public override int GetHashCode() => this.Index;
    }
}
