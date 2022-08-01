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
            //利用可能なサポートを大きい順に並べておく
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
            Console.WriteLine($"{plans.Length}個の組立順候補を生成しました．最良の組立順を決定します...");
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
            //組立中の部品．各段の全ブロックが組み立てられなかった場合は，その段の組立可能ブロックもこの変数に入る
            var assembling = new Assembly();
            //組立順が確定した部品．各段の全ブロックが組み立てられなかった場合は，その段の組立可能ブロックはこの変数に入らない
            var assembled = new Assembly();
            //組み立てられないブロックをここに記録する
            IReadOnlyCollection<AssemblyComponent> unassembledComponents = new List<AssemblyComponent>();
            while (true)
            {
                //未組立ブロックのうち，最下層にあるものを取り出す
                var remainingLayer = remainingLayers.First();
                //1層組立
                this.AssembleLayer(remainingLayer, assembled, out assembling, out unassembledComponents);
                //組み立てられなかったブロックが存在する場合
                if (unassembledComponents.Any())
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"{header}{remainingLayer.Key}段目に組み立て不可能なブロックが{unassembledComponents.Count}個あります");
                    Console.ForegroundColor = ConsoleColor.White;
                    break;
                }
                //最後の段まで組み立て終わった場合
                else if (remainingLayers.Count() == 1)
                {
                    Console.WriteLine($"{header}{remainingLayer.Key}段目の全ブロック({remainingLayer.Count()}個)を組み立てました");
                    Console.ForegroundColor = ConsoleColor.Cyan;
                    Console.WriteLine($"{header}組立順候補を生成しました．サブアセンブリ:{currentAssemblies.Count} サポート:{supportCountSum}");
                    Console.ForegroundColor = ConsoleColor.White;
                    currentAssemblies.Add(assembling);
                    this.minSupportCount = supportCountSum;
                    yield return new AssemblyPlan(currentAssemblies);
                    yield break;
                }
                //この段の全ブロックを組み立てられた場合
                else
                {
                    Console.WriteLine($"{header}{remainingLayer.Key}段目の全ブロック({remainingLayer.Count()}個)を組み立てました");
                    assembled = assembling;
                    remainingLayers = remainingLayers.Skip(1).ToArray();
                }
            }
            //ここに到達したということは，組み立てられなかったブロックが存在するということ．ただしブロック同士の衝突といったどうやっても組み立てられないパターンは存在しない
            //サブアセンブリを作成するプランへ分岐．ただし，高さが規定値未満のサブアセンブリは作成されないように弾く
            var subassemblizationCondition = assembling.TopPosition >= subassemblizeableLayer;
            var layerCondition = assembled.TopPosition - assembled.BottomPosition >= this.MinSubassemblyLayerCount - 1;
            var subassemblyCountCondition = currentAssemblies.Count + 1 < this.MaxSubassemblyCount;
            if (subassemblizationCondition && layerCondition && subassemblyCountCondition)
            {
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine($"{header}{assembled.TopPosition + 1}段目からサブアセンブリ分割プランへ分岐します");
                Console.ForegroundColor = ConsoleColor.White;
                var subassemblyPlanCurrentAssemblies = currentAssemblies.Append(assembled).ToList();
                var subassemblyPlans = this.GenerateCandidates(subassemblyPlanCurrentAssemblies, currentSandwichedSupportCount, remainingLayers, recursive + 1, assembled.TopPosition, supportCountSum).ToArray();
                foreach (var subassemblyPlan in subassemblyPlans) yield return subassemblyPlan;
            }
            else
            {
                Console.ForegroundColor = ConsoleColor.DarkYellow;
                Console.WriteLine($"{header}サブアセンブリ分割プランへ分岐しません");
                Console.ForegroundColor = ConsoleColor.White;
            }
            //サポートを追加するプランへ分岐
            var supportPlanCurrentAssemblies = currentAssemblies.ToList();
            IEnumerable<IGrouping<int, AssemblyComponent>> supportPlanRemainingLayers;
            var supportPlanSandwichedSupportCount = currentSandwichedSupportCount;
            supportPlanRemainingLayers = GenerateRemainingLayersOfSupportPlan(assembling, unassembledComponents, remainingLayers.Skip(1), ref supportPlanSandwichedSupportCount, out var supportCount).ToArray();
            //間に挟まれたサポートが規定数を超えるなら，この分岐をキャンセル
            if (supportPlanSandwichedSupportCount > this.MaxSandwichedSupportCount)
            {
                Console.ForegroundColor = ConsoleColor.DarkGreen;
                Console.WriteLine($"{header}間に挟まれたサポートが多すぎます");
                Console.ForegroundColor = ConsoleColor.White;
                yield break;
            }
            //サポート数がより少ない組立計画がすでに生成されていた場合は，この分岐をキャンセル
            if (supportCountSum + supportCount > this.minSupportCount)
            {
                Console.ForegroundColor = ConsoleColor.DarkGreen;
                Console.WriteLine($"{header}よりサポートが少ない組立計画がすでに生成されています");
                Console.ForegroundColor = ConsoleColor.White;
                yield break;
            }
            if (supportCount == 0)
            {
                Console.ForegroundColor = ConsoleColor.DarkGreen;
                Console.WriteLine($"{header}サポートを追加できませんでした");
                Console.ForegroundColor = ConsoleColor.White;
                yield break;
            }
            //
            var remainingLayerMin = supportPlanRemainingLayers.Min(layer => layer.Key);
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine($"{header}サポート追加プランへ分岐します．サポートを追加して{remainingLayerMin}段目から組み立て直します．これまで:{supportCountSum} 追加:{supportCount} 挟:{supportPlanSandwichedSupportCount}");
            Console.ForegroundColor = ConsoleColor.White;
            var supportPlans = this.GenerateCandidates(supportPlanCurrentAssemblies, supportPlanSandwichedSupportCount, supportPlanRemainingLayers, recursive + 1, assembling.TopPosition + 1, supportCountSum + supportCount).ToArray();
            foreach (var supportPlan in supportPlans) yield return supportPlan;
        }
        /// <summary>
        /// ブロック1層を組み立てる．
        /// </summary>
        /// <param name="remainingLayer">組み立てるブロック層．</param>
        /// <param name="assembly">この部品にブロックを組み立てる．</param>
        /// <param name="assembled">組立可能なブロックを組み立てた部品．</param>
        /// <param name="unassembledComponents">組み立てられなかったブロック．</param>
        private void AssembleLayer(IEnumerable<AssemblyComponent> remainingLayer, Assembly assembly, out Assembly assembled, out IReadOnlyCollection<AssemblyComponent> unassembledComponents)
        {
            var assembling = new Assembly(assembly);
            var remainingComponents = remainingLayer.ToList();
            while (true)
            {
                //未組立のブロックすべての組立可能性を計算．組立可能なものを組み立てやすさ順に並び替える
                //組み立てやすさの評価値が同じもの同士は位置順に並び替える
                var assemblableComponents = remainingComponents.AsParallel()
                .SelectWithSource(c => this.assemblabilityProvider.GetAssemblability(assembling, c))
                .Where(tuple => tuple.map.IsAssemblable)
                .OrderByDescending(tuple => tuple.map)
                .ThenBy(tuple => tuple.source.Position.X)
                .ThenBy(tuple => tuple.source.Position.Y)
                .Select(tuple => tuple.source)
                .ToArray();
                //組立可能なブロックがなくなったらループ終了
                if (!assemblableComponents.Any()) break;
                //組立可能なブロックを組み立てる
                foreach (var component in assemblableComponents)
                {
                    assembling.AddComponent(component);
                }
                //組み立てたブロックを未組立ブロックのリストから削除
                remainingComponents = remainingComponents.Except(assemblableComponents, componentSizeAndPositionComparer).ToList();
            }
            //組立結果を渡す
            assembled = assembling;
            //組み立てられなかったブロックを渡す
            unassembledComponents = remainingComponents;
        }
        /// <summary>
        /// サポートブロックを追加するプランにて組み立てる必要のあるブロック層を返す．
        /// </summary>
        /// <param name="assembly">サポートブロックを追加したい組み立てかけの部品．</param>
        /// <param name="unassembledComponents">組み立て不可能と判定されたブロック．</param>
        /// <param name="remainingLayers">まだ組立可能性の判定を行っていないブロック層．</param>
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
        /// 指定した組立不可能ブロックを組み立てるためのサポート柱を返す．
        /// </summary>
        /// <param name="assemblyWithSupport">サポートブロックを追加していく部品．この内容は上書きされる．</param>
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
        /// 組み立て不可能ブロックを支持するためのサポートブロックを返す．
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
            //ここに到達したらサポートを使用した組立が不可能ということになる
            supportComponent = null;
            return false;
        }
    }
}
