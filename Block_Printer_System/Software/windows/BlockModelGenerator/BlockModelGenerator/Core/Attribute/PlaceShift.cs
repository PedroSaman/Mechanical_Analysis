using Core.Geometry;

namespace Core.Attribute
{
    /// <summary>
    /// ブロックをプレイスするとき，アプローチ位置をずらすか表す．
    /// </summary>
    readonly struct PlaceShift : IAssemblyAttribute
    {
        public readonly Grid2 Shift;
        public PlaceShift(int shiftX, int shiftY) => this.Shift = new Grid2(shiftX, shiftY);
        public bool Equals(IAssemblyAttribute other) => other is PlaceShift shift && this.Shift.Equals(shift.Shift);
        public override int GetHashCode() => this.Shift.GetHashCode();
    }
}
