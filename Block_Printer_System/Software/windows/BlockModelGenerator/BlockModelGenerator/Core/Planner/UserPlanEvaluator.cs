using System;
using System.Collections.Generic;
using System.Linq;
using Core.Attribute;

namespace Core.Planner
{
    class UserPlanEvaluator : IAssemblyPlanEvaluator
    {
        class Evaluation
        {
            public readonly int AssemblyCount;
            public readonly IReadOnlyCollection<int> BlockCounts;
            public int BlockCount => this.BlockCounts.Sum();
            public readonly IReadOnlyCollection<int> SupportCounts;
            public int SupportCount => this.SupportCounts.Sum();
            public double SupportRatio => this.SupportCount / (double)this.BlockCount;
            public Evaluation(AssemblyPlan plan)
            {
                this.AssemblyCount = plan.Count();
                this.BlockCounts = plan
                    .Select(assembly => assembly.Count())
                    .ToArray();
                this.SupportCounts = plan
                    .Select(assembly => assembly.Count(block => block.BlockAttributeCollection.Exists<Support>()))
                    .ToArray();
            }
        }
        public AssemblyPlan SelectBestPlan(IEnumerable<AssemblyPlan> plans)
        {
            var planEvaluations =
                (from plan in plans
                let evaluation = new Evaluation(plan)
                group (plan, evaluation) by evaluation.AssemblyCount)
                .ToArray();
            var userEvaluations = new List<(AssemblyPlan plan, int evaluation)>();
            foreach (var (planEvaluation,index) in planEvaluations.WithIndex())
            {
                var order =
                    from tuple in planEvaluation
                    orderby tuple.evaluation.SupportRatio
                    select tuple;
                var (plan, evaluation) = order.First();
                Console.WriteLine($"\n[{index + 1}/{planEvaluations.Length}]:");
                Console.WriteLine($"部品数:{evaluation.AssemblyCount}");
                Console.WriteLine($"ブロック数:{evaluation.BlockCount} ({evaluation.BlockCounts.Aggregate("", (current, next) => current + "," + next).Trim(',')})");
                Console.WriteLine($"サポートブロック数:{evaluation.SupportCount} ({evaluation.SupportCounts.Aggregate("", (current, next) => current + "," + next).Trim(',')})");
                while (true)
                {
                    Console.Write("この組立計画の評価値を入力:");
                    try
                    {
                        var userEvaluation = int.Parse(Console.ReadLine());
                        userEvaluations.Add((plan, userEvaluation));
                        break;
                    }
                    catch
                    {
                        Console.WriteLine("評価値には整数を入力してください");
                    }
                }
            }
            return userEvaluations.OrderByDescending(tuple => tuple.evaluation).First().plan;
        }
    }
}
