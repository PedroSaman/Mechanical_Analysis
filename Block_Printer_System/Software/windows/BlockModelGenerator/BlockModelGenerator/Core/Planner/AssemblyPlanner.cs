using System;
using System.Collections.Generic;
using System.Linq;
using Core.Geometry;
using Core.Attribute;

namespace Core.Planner
{
    class AssemblyPlanner : IAssemblyPlanner
    {
        private static readonly ClosureEqualityComparer<AssemblyComponent> componentSizeAndPositionComparer = new ClosureEqualityComparer<AssemblyComponent>(
            (a1, a2) => a1.Size.Equals(a2.Size) && a1.Position.Equals(a2.Position)
            );
        private readonly IAssemblabilityEvaluator assemblabilityProvider;
        private readonly IAssemblyPlanEvaluator assemblyPlanEvaluator;
        private readonly IReadOnlyCollection<Grid3> availableSupportBlockSizes;
        public int MaxSandwichedSupportCount { get; set; } = int.MaxValue;
        public int MaxSubassemblyCount { get; set; } = int.MaxValue;
        public int MinSubassemblyLayerCount { get; set; } = 2;
        private int minSupportCount;
        public AssemblyPlanner(IAssemblabilityEvaluator assemblabilityProvider, IAssemblyPlanEvaluator assemblyPlanEvaluator, IEnumerable<Grid3> availableSupportBlockSizes)
        {
            this.assemblabilityProvider = assemblabilityProvider;
            this.assemblyPlanEvaluator = assemblyPlanEvaluator;
            //List available supports in order of largest to smallest
            this.availableSupportBlockSizes = availableSupportBlockSizes
                .OrderByDescending(size => size.Z)
                .ThenByDescending(size => size.X + size.Y)
                .ToArray();
        }
        public AssemblyPlan GeneratePlan(IEnumerable<AssemblyComponent> components)
        {
            this.minSupportCount = int.MaxValue;
            var blockLayers = components
                .GroupBy(component => component.TopPosition)
                .OrderBy(group => group.Key)
                .ToArray();
            var plans = this.GenerateCandidates(new List<Assembly>(), 0, blockLayers, 0, int.MinValue, 0).ToArray();
            Console.WriteLine($"Generated {plans.Length} candidates. Determine the best assembly order...");
            return this.assemblyPlanEvaluator.SelectBestPlan(plans);
        }
        private IEnumerable<AssemblyPlan> GenerateCandidates(List<Assembly> currentAssemblies,
            int currentSandwichedSupportCount,
            IEnumerable<IGrouping<int, AssemblyComponent>> remainingLayers,
            int recursive,
            int subassemblizeableLayer,
            int supportCountSum)
        {
            var header = new string(Enumerable.Repeat(' ', recursive).ToArray());
            // Parts under assembly. If all blocks in each tier have not been assembled, the assemblable blocks in that tier will also be entered into this variable
            var assembling = new Assembly();
            // Parts for which the assembly order has been determined. If all blocks in each tier are not assembled, the assembleable blocks in that tier do not enter this variable
            var assembled = new Assembly();
            // Record here the blocks that cannot be assembled
            IReadOnlyCollection<AssemblyComponent> unassembledComponents = new List<AssemblyComponent>();
            while (true)
            {
                // Remove the bottom layer of unassembled blocks
                var remainingLayer = remainingLayers.First();
                // Single layer assembly
                this.AssembleLayer(remainingLayer, assembled, out assembling, out unassembledComponents);
                //If there are blocks that could not be assembled
                if (unassembledComponents.Any())
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"There are {unassembledComponents.Count} blocks in the {header}{remainingLayer.Key} stage that cannot be assembled.");
                    Console.ForegroundColor = ConsoleColor.White;
                    break;
                }
                // If you have finished assembling to the last step
                else if (remainingLayers.Count() == 1)
                {
                    Console.WriteLine($"All blocks ({remainingLayer.Count()}) of the {header}{remainingLayer.Key} stage have been assembled");
                    Console.ForegroundColor = ConsoleColor.Cyan;
                    Console.WriteLine($"{header} assembly order candidates generated. Subassemblies:{currentAssemblies.Count} Support:{supportCountSum}");
                    Console.ForegroundColor = ConsoleColor.White;
                    currentAssemblies.Add(assembling);
                    this.minSupportCount = supportCountSum;
                    yield return new AssemblyPlan(currentAssemblies);
                    yield break;
                }
                // If all blocks in this tier are assembled
                else
                {
                    Console.WriteLine($"All blocks ({remainingLayer.Count()}) of the {header}{remainingLayer.Key} stage have been assembled");
                    assembled = assembling;
                    remainingLayers = remainingLayers.Skip(1).ToArray();
                }
            }
            // The fact that we have reached this point means that there are blocks that could not be assembled. However, there are no patterns such as collisions between blocks that cannot be assembled in any way.
            // Branches to a plan to create subassemblies. However, subassemblies with a height less than the specified value are not created.
            var subassemblizationCondition = assembling.TopPosition >= subassemblizeableLayer;
            var layerCondition = assembled.TopPosition - assembled.BottomPosition >= this.MinSubassemblyLayerCount - 1;
            var subassemblyCountCondition = currentAssemblies.Count + 1 < this.MaxSubassemblyCount;
            if (subassemblizationCondition && layerCondition && subassemblyCountCondition)
            {
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine($"{header}{assembled.TopPosition + 1} branch from the first row to the subassembly division plan");
                Console.ForegroundColor = ConsoleColor.White;
                var subassemblyPlanCurrentAssemblies = currentAssemblies.Append(assembled).ToList();
                var subassemblyPlans = this.GenerateCandidates(subassemblyPlanCurrentAssemblies, currentSandwichedSupportCount, remainingLayers, recursive + 1, assembled.TopPosition, supportCountSum).ToArray();
                foreach (var subassemblyPlan in subassemblyPlans) yield return subassemblyPlan;
            }
            else
            {
                Console.ForegroundColor = ConsoleColor.DarkYellow;
                Console.WriteLine($"{header}Does not branch to subassembly split plan");
                Console.ForegroundColor = ConsoleColor.White;
            }
            // Branch to plan to add support
            var supportPlanCurrentAssemblies = currentAssemblies.ToList();
            IEnumerable<IGrouping<int, AssemblyComponent>> supportPlanRemainingLayers;
            var supportPlanSandwichedSupportCount = currentSandwichedSupportCount;
            supportPlanRemainingLayers = GenerateRemainingLayersOfSupportPlan(assembling, unassembledComponents, remainingLayers.Skip(1), ref supportPlanSandwichedSupportCount, out var supportCount).ToArray();
            // Cancel this branch if the number of intervening supports exceeds the specified number
            if (supportPlanSandwichedSupportCount > this.MaxSandwichedSupportCount)
            {
                Console.ForegroundColor = ConsoleColor.DarkGreen;
                Console.WriteLine($"{header} there are too much support blocks in between!");
                Console.ForegroundColor = ConsoleColor.White;
                yield break;
            }
            // Cancel this branch if an assembly plan with fewer supports has already been generated
            if (supportCountSum + supportCount > this.minSupportCount)
            {
                Console.ForegroundColor = ConsoleColor.DarkGreen;
                Console.WriteLine($"Assembly plans with less support blocks than {header} have already been generated");
                Console.ForegroundColor = ConsoleColor.White;
                yield break;
            }
            if (supportCount == 0)
            {
                Console.ForegroundColor = ConsoleColor.DarkGreen;
                Console.WriteLine($"Failed to add {header} support");
                Console.ForegroundColor = ConsoleColor.White;
                yield break;
            }
            //
            var remainingLayerMin = supportPlanRemainingLayers.Min(layer => layer.Key);
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine($"Branch to the {header} plan for adding support. Add support and reassemble from the {remainingLayerMin} stage. So far:{supportCountSum} add:{supportCount} pinch:{supportPlanSandwichedSupportCount}");
            Console.ForegroundColor = ConsoleColor.White;
            var supportPlans = this.GenerateCandidates(supportPlanCurrentAssemblies, supportPlanSandwichedSupportCount, supportPlanRemainingLayers, recursive + 1, assembling.TopPosition + 1, supportCountSum + supportCount).ToArray();
            foreach (var supportPlan in supportPlans) yield return supportPlan;
        }
        /// <summary>
        /// Assemble one layer of blocks.
        /// </summary>
        /// <param name="remainingLayer">Block layer to assemble. </param>
        /// <param name="assembly">Block assembly on this part. </param>
        /// <param name="assembled">Assembled part with blocks that can be assembled. </param>
        /// <param name="unassembledComponents">Blocks that were not assembled. </param>
        private void AssembleLayer(IEnumerable<AssemblyComponent> remainingLayer, Assembly assembly, out Assembly assembled, out IReadOnlyCollection<AssemblyComponent> unassembledComponents)
        {
            var assembling = new Assembly(assembly);
            var remainingComponents = remainingLayer.ToList();
            while (true)
            {
                //Calculates the assembleability of all unassembled blocks. Sort the assembleables by ease of assembly.
                //Sort items with the same ease-of-assembly evaluation value in order of position.
                var assemblableComponents = remainingComponents.AsParallel()
                .SelectWithSource(c => this.assemblabilityProvider.GetAssemblability(assembling, c))
                .Where(tuple => tuple.map.IsAssemblable)
                .OrderByDescending(tuple => tuple.map)
                .ThenBy(tuple => tuple.source.Position.X)
                .ThenBy(tuple => tuple.source.Position.Y)
                .Select(tuple => tuple.source)
                .ToArray();
                //Loop ends when there are no more blocks available for assembly.
                if (!assemblableComponents.Any()) break;
                //Assemble blocks that can be assembled.
                foreach (var component in assemblableComponents)
                {
                    assembling.AddComponent(component);
                }
                //Remove assembled blocks from the list of unassembled blocks
                remainingComponents = remainingComponents.Except(assemblableComponents, componentSizeAndPositionComparer).ToList();
            }
            //Passing on assembly results
            assembled = assembling;
            //Passing on blocks that were not assembled.
            unassembledComponents = remainingComponents;
        }
        /// <summary>
        /// Returns the block layers that need to be assembled in the plan to add support blocks.
        /// </summary>
        /// <param name="assembly">The part to be assembled to which you want to add support blocks. </param>
        /// <param name="unassembledComponents">Blocks that are determined to be unassembled. </param>
        /// <param name="remainingLayers">Block layers that have not yet been determined to be ready for assembly. </param>
        /// <returns></returns>
        private IEnumerable<IGrouping<int, AssemblyComponent>> GenerateRemainingLayersOfSupportPlan(Assembly assembly, IEnumerable<AssemblyComponent> unassembledComponents, IEnumerable<IGrouping<int, AssemblyComponent>> remainingLayers, ref int sandwichedSupportCount, out int supportCount)
        {
            //サポートを追加していく臨時部品を作成
            var assemblyWithSupport = new Assembly(assembly);
            var supportComponents = new List<AssemblyComponent>();
            //組立不可能ブロックのためのサポートを求める
            foreach (var unassemblableComponent in unassembledComponents)
            {
                supportComponents.AddRange(this.GetSupportBlockTowerFor(assemblyWithSupport, unassemblableComponent, ref sandwichedSupportCount));
            }
            //サポート追加プランに登場する全ブロックを列挙し，層ごとに分類して返す
            var remainingBlocks = remainingLayers.SelectMany(group => group);
            var supportPlanBlocks = assembly.Concat(unassembledComponents)
                .Concat(supportComponents)
                .Concat(remainingBlocks);
            supportCount = supportComponents.Count;
            return supportPlanBlocks.ToLookup(block => block.TopPosition)
                .OrderBy(group => group.Key);
        }
        /// <summary>
        /// Returns the support columns for assembling the specified unassembled block.
        /// </summary>
        /// <param name="assemblyWithSupport">The component to which the support block will be added. This content is overwritten. </param>
        /// <param name="unassemblableComponent"></param>
        /// <returns></returns>
        private IReadOnlyCollection<AssemblyComponent> GetSupportBlockTowerFor(Assembly assemblyWithSupport, AssemblyComponent unassemblableComponent, ref int sandwichedSupportCount)
        {
            var supportComponents = new List<AssemblyComponent>();
            //サポート柱が底面に届くか他の部品構成ブロックにぶつかるまで，サポート柱を上から下に伸ばす
            //サポートブロック上面の座標．
            var supportTopPosition = unassemblableComponent.BottomPosition - 1;
            while (true)
            {
                if (!this.TryGetSupportBlockFor(assemblyWithSupport, unassemblableComponent, supportTopPosition, out var supportComponent))
                {
                    //サポートを追加できなかった場合はもう一段下からやり直してみる
                    supportTopPosition--;
                    continue;
                }
                assemblyWithSupport.AddComponent(supportComponent);
                supportComponents.Add(supportComponent);
                //底面に届いたら終了
                var supportBottomPosition = supportTopPosition - supportComponent.Size.Z + 1;
                if (supportBottomPosition == assemblyWithSupport.BottomPosition) break;
                //まだ底面に届いておらず，かつ部品中の他のブロックとどうしてもぶつかるなら終了
                //これは，部品ブロックの間に挟まれたサポートブロックとなる
                var planarBelowPositions = unassemblableComponent.BottomPositions.Select(p => new Grid3(p.X, p.Y, supportBottomPosition - 1));
                if (planarBelowPositions.All(p => assemblyWithSupport.Occupy(p)))
                {
                    sandwichedSupportCount += supportComponents.Count;
                    break;
                }
                //ひとつ下の段のサポート追加へ
                supportTopPosition -= supportComponent.Size.Z;
            }
            return supportComponents;
        }
        /// <summary>
        /// Returns a support block to support the unassembled block.
        /// </summary>
        /// <param name="assembly"></param>
        /// <param name="unassemblableComponent"></param>
        /// <param name="supportTopPosition">サポートブロックの上面位置．</param>
        /// <returns></returns>
        private bool TryGetSupportBlockFor(Assembly assembly, AssemblyComponent unassemblableComponent, int supportTopPosition, out AssemblyComponent supportComponent)
        {
            //サポートブロックのサイズごとに判定
            foreach (var supportSize in this.availableSupportBlockSizes)
            {
                var supportBelowPosition = new Grid3(unassemblableComponent.LeftPosition, unassemblableComponent.BackPosition, supportTopPosition - supportSize.Z + 1);
                var planerBelowPositions = unassemblableComponent.BottomPositions.Select(p => new Grid2(p.X, p.Y));
                //サポートブロックの位置を求める．これは，サポートブロックが組立不可能ブロックのx-y投影領域からはみ出ない位置に限定される
                var offsets =
                    from x in Enumerable.Range(0, unassemblableComponent.Size.X - supportSize.X + 1)
                    from y in Enumerable.Range(0, unassemblableComponent.Size.Y - supportSize.Y + 1)
                    select new Grid3(x, y, 0);
                foreach (var offset in offsets)
                {
                    var supportPosition = supportBelowPosition + offset;
                    var blockAttributeCollection = new GenericCollection<IBlockAttribute>(new IBlockAttribute[] { Support.Instance });
                    var assemblyAttributeCollection = GenericCollection.Empty<IAssemblyAttribute>();
                    var supportCandidate = new AssemblyComponent(supportSize, blockAttributeCollection, supportPosition, assemblyAttributeCollection);
                    var planarSupportPositions = supportCandidate.TopPositions.Select(p => new Grid2(p.X, p.Y));
                    //サポートが組立不可能ブロックのx-y投影領域内に入っており，かつ部品と干渉しなければ返す
                    if (planarSupportPositions.All(p => planerBelowPositions.Contains(p) && !assembly.ConflictWith(supportCandidate)))
                    {
                        supportComponent = supportCandidate;
                        return true;
                    }
                }
            }
            // Once you reach this point, it means that assembly with supports is not possible.
            supportComponent = null;
            return false;
        }
    }
}
