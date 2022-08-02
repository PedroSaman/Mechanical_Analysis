using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Core.Geometry;
using Core.Attribute;

namespace Core
{
    /// <summary>
    /// Represents a component consisting of multiple blocks and the order in which they are assembled.
    /// </summary>
    class Assembly : IEnumerable<AssemblyComponent>
    {
        private readonly List<AssemblyComponent> components;
        private readonly Dictionary<int, List<AssemblyComponent>> upperLayers;
        private readonly Dictionary<int, List<AssemblyComponent>> lowerLayers;
        private int? rightPosition;
        private int? leftPosition;
        private int? frontPosition;
        private int? backPosition;
        private int? topPosition;
        private int? bottomPosition;
        public int RightPosition
        {
            get => this.rightPosition.Value;
            private set => this.rightPosition = value;
        }
        public int LeftPosition
        {
            get => this.leftPosition.Value;
            private set => this.leftPosition = value;
        }
        public int FrontPosition
        {
            get => this.frontPosition.Value;
            private set => this.frontPosition = value;
        }
        public int BackPosition
        {
            get => this.backPosition.Value;
            private set => this.backPosition = value;
        }
        public int TopPosition
        {
            get => this.topPosition.Value;
            private set => this.topPosition = value;
        }
        public int BottomPosition
        {
            get => this.bottomPosition.Value;
            private set => this.bottomPosition = value;
        }
        /// <summary>
        /// 空の状態で新しい部品を初期化する．
        /// </summary>
        public Assembly()
        {
            this.components = new List<AssemblyComponent>();
            this.upperLayers = new Dictionary<int, List<AssemblyComponent>>();
            this.lowerLayers = new Dictionary<int, List<AssemblyComponent>>();
        }
        /// <summary>
        /// 順序付けされたブロックを用いて新しい部品を初期化する．
        /// </summary>
        /// <param name="components"></param>
        public Assembly(IEnumerable<AssemblyComponent> components) : this()
        {
            this.components = new List<AssemblyComponent>();
            foreach (var component in components) this.AddComponent(component);
        }
        public bool Occupy(Grid3 position) => this.upperLayers.ContainsKey(position.Z) && this.upperLayers[position.Z].Any(b => b.Occupy(position));
        public IReadOnlyList<AssemblyComponent> GetComponentsByUpperPosition(int upperPosition) =>
            this.upperLayers.ContainsKey(upperPosition) ? this.upperLayers[upperPosition] : new List<AssemblyComponent>();
        public IReadOnlyList<AssemblyComponent> GetComponentsByLowerPosition(int lowerPosition) =>
            this.lowerLayers.ContainsKey(lowerPosition) ? this.lowerLayers[lowerPosition] : new List<AssemblyComponent>();
        /// <summary>
        /// この部品の末尾に指定したブロックを追加する．
        /// </summary>
        /// <param name="component"></param>
        /// <exception cref="InvalidOperationException"></exception>
        public void AddComponent(AssemblyComponent component)
        {
            if (component.OccupyingPositions.Any(p => this.Occupy(p)))
            {
                var message = $"cannot add a block due to confliction. size:({component.Size.X},{component.Size.Y},{component.Size.Z}) position:({component.Position.X},{component.Position.Y},{component.Position.Z})";
                var conflictComponents = this.components.Where(b => b.OccupyingPositions.Intersect(component.OccupyingPositions).Any());
                message += "\nconflict components:";
                foreach (var c in conflictComponents)
                {
                    message += $"\nsize:({component.Size.X},{component.Size.Y},{component.Size.Z}) position:({component.Position.X},{component.Position.Y},{component.Position.Z})";
                }
                throw new InvalidOperationException(message);
            }
            //Change assembly attributes
            var assemblyAttributes = component.AssemblyAttributeCollection
                .Append(this.GetPlaceShiftFor(component))
                .Append(this.GetPressCapabilityFor(component));
            var addedComponent = new AssemblyComponent(component.Size, component.BlockAttributeCollection, component.Position, assemblyAttributes);
            //Add block
            this.components.Add(addedComponent);
            if (!this.upperLayers.ContainsKey(addedComponent.TopPosition)) this.upperLayers.Add(addedComponent.TopPosition, new List<AssemblyComponent>());
            this.upperLayers[addedComponent.TopPosition].Add(addedComponent);
            if (!this.lowerLayers.ContainsKey(addedComponent.BottomPosition)) this.lowerLayers.Add(addedComponent.BottomPosition, new List<AssemblyComponent>());
            this.lowerLayers[addedComponent.BottomPosition].Add(addedComponent);
            //Update part end position
            this.RightPosition = Math.Max(this.rightPosition ?? int.MinValue, addedComponent.RightPosition);
            this.LeftPosition = Math.Min(this.leftPosition ?? int.MaxValue, addedComponent.LeftPosition);
            this.FrontPosition = Math.Max(this.frontPosition ?? int.MinValue, addedComponent.FrontPosition);
            this.BackPosition = Math.Min(this.backPosition ?? int.MaxValue, addedComponent.BackPosition);
            this.TopPosition = Math.Max(this.topPosition ?? int.MinValue, addedComponent.TopPosition);
            this.BottomPosition = Math.Min(this.bottomPosition ?? int.MaxValue, addedComponent.BottomPosition);
        }
        /// <summary>
        /// ブロックプレイス位置のシフト係数を決定する
        /// </summary>
        /// <param name="component"></param>
        /// <returns></returns>
        private PlaceShift GetPlaceShiftFor(AssemblyComponent component)
        {
            var rightNeightbors = component.RightPositions.Select(p => p + Grid3.XUnit);
            var leftNeightbors = component.LeftPositions.Select(p => p - Grid3.XUnit);
            var frontNeightbors = component.FrontPositions.Select(p => p + Grid3.YUnit);
            var backNeightbors = component.BackPositions.Select(p => p - Grid3.YUnit);
            int shiftX = 0;
            int shiftY = 0;
            if (rightNeightbors.Any(p => this.Occupy(p))) shiftX--;
            if (leftNeightbors.Any(p => this.Occupy(p))) shiftX++;
            if (frontNeightbors.Any(p => this.Occupy(p))) shiftY--;
            if (backNeightbors.Any(p => this.Occupy(p))) shiftY++;
            return new PlaceShift(shiftX, shiftY);
        }
        /// <summary>
        /// 指定したブロックを組み立てるための安定土台を有するか返す．
        /// </summary>
        /// <param name="copmonent"></param>
        /// <returns></returns>
        private PressCapability GetPressCapabilityFor(AssemblyComponent copmonent)
        {
            //最初に組み立てるブロックなら必ず安定状態とする
            if (!this.Any()) return PressCapability.CanPress;
            //下から2段目までは安定状態とする
            if (copmonent.Position.Z <= this.BottomPosition + 1) return PressCapability.CanPress;
            //ブロックの下の座標を列挙する
            var basePositions =
                from z in EnumerableExtension.FromTo(this.BottomPosition, copmonent.Position.Z - 1)
                let possession = copmonent.BottomPositions
                let checkPositions = possession.Select(p => new Grid3(p.X, p.Y, z))
                select checkPositions;
            //列挙した座標すべてが他のブロックによって占有されていれば安定土台と判定する
            return basePositions
                .SelectMany(positions => positions)
                .All(p => this.Occupy(p))
                ? PressCapability.CanPress : PressCapability.CannotPress;
        }
        public IEnumerator<AssemblyComponent> GetEnumerator() => this.components.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => this.components.GetEnumerator();
    }
}
