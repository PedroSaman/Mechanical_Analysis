namespace Core.Attribute
{
    /// <summary>
    /// 組立場所番号を表す．
    /// </summary>
    readonly struct AssemblyArea : IAssemblyAttribute
    {
        public static readonly AssemblyArea Default = new AssemblyArea(0);
        public readonly int Index;
        private AssemblyArea(int index) => this.Index = index;
        /// <summary>
        /// この組立場所の次の組立場所を取得する．
        /// </summary>
        /// <returns></returns>
        public AssemblyArea Next() => new AssemblyArea(this.Index + 1);
        public bool Equals(IAssemblyAttribute other) => other is AssemblyArea area && this.Index == area.Index;
        public override int GetHashCode() => this.Index;
    }
}
