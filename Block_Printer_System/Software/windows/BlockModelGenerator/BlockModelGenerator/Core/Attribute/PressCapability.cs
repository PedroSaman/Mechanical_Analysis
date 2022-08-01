namespace Core.Attribute
{
    /// <summary>
    /// このブロックの組立時，ブロックを強く押し込むことができるか示す．
    /// </summary>
    readonly struct PressCapability : IAssemblyAttribute
    {
        public static readonly PressCapability CanPress = new PressCapability(true);
        public static readonly PressCapability CannotPress = new PressCapability(false);
        public readonly bool Capability;
        private PressCapability(bool canPress) => this.Capability = canPress;
        public bool Equals(IAssemblyAttribute other) => other is PressCapability press && this.Capability == press.Capability;
        public override int GetHashCode() => this.Capability.GetHashCode();
    }
}
