using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using Core;
using Core.Planner;
using Core.Serialization;
using Core.Attribute;
using Core.Geometry;
using System.Runtime.Serialization.Json;

namespace BlockModelGenerator
{
    class Program
    {
        private static PlannerSetting GetProgramSetting()
        {
            var settingFilename = Directory.GetCurrentDirectory() + "/AssemblyPlanSetting.json";
            var jsonSetting = new DataContractJsonSerializerSettings
            {
                UseSimpleDictionaryFormat = true
            };
            var serializer = new DataContractJsonSerializer(typeof(PlannerSetting), jsonSetting);
            try
            {
                using (var reader = new StreamReader(settingFilename))
                {
                    return serializer.ReadObject(reader.BaseStream) as PlannerSetting;
                }
            }
            catch
            {
                Console.WriteLine("設定ファイルを開けませんでした．規定の設定ファイルを作成します");
                try
                {
                    using (var fileWriter = new FileStream(settingFilename, FileMode.Create))
                    {
                        using (var jsonWriter = JsonReaderWriterFactory.CreateJsonWriter(fileWriter, Encoding.UTF8, true, true, "  "))
                        {
                            serializer.WriteObject(jsonWriter, PlannerSetting.Default);
                        }
                    }
                    Console.WriteLine("規定の設定ファイルを作成しました");
                }
                catch (Exception e)
                {
                    Console.WriteLine($"設定ファイルを作成できませんでした:{e.Message}");
                }
                return PlannerSetting.Default;
            }
        }
        private static void DisplayBlockGroup(IEnumerable<Block> blocks)
        {
            var blockGroup = blocks
            .GroupBy(b => b)
            .OrderByDescending(g => g.Count());
            foreach (var g in blockGroup)
            {
                var size = $"{g.Key.Size.X}x{g.Key.Size.Y}x{g.Key.Size.Z}";
                var count = g.Count();
                if (g.Key.BlockAttributeCollection.Exists<Color>())
                {
                    var color = g.Key.BlockAttributeCollection.Single<Color>().Index;
                    Console.WriteLine($"サイズ{size} 色{color}のブロックは{count}個必要です．");
                }
                else if (g.Key.BlockAttributeCollection.Exists<Support>())
                {
                    Console.WriteLine($"サイズ{size}のサポートブロックは{count}個必要です．");
                }
                else
                {
                    Console.WriteLine("unexpected block status");
                    continue;
                }
            }
        }
        static void Main(string[] args)
        {
            if (!args.Any())
            {
                Console.WriteLine("このプログラムには1つ以上のファイルを入力として渡してください");
                return;
            }
            var setting = GetProgramSetting();
            IComponentCollectionReader reader = new ComponentCollectionReader();
            IAssemblabilityEvaluator assemblabilityEvaluator = new LocalAssemblabilityEvaluator();
            IAssemblyPlanEvaluator planEvaluator = new UserPlanEvaluator();
            IAssemblyPlanner planner = new AssemblyPlanner(assemblabilityEvaluator, planEvaluator, setting.AvailableSupportBlockSizes)
            {
                MaxSubassemblyCount = setting.MaxSubassemblyCount,
                MinSubassemblyLayerCount = setting.MinSubassemblyLayerCount,
            };
            IAssemblyPlanWriter writer = new AssemblyPlanWriter();
            foreach (var (input, index) in args.WithIndex())
            {
                var header = $"[{index + 1}/{args.Length}] ";
                try
                {
                    Console.WriteLine($"{header}{input}を読み込みます..");
                    var components = reader.ReadComponentsFrom(input);
                    Console.WriteLine($"{header}{input}を読み込みました．ブロック数:{components.Count()}");
                    var plan = planner.GeneratePlan(components);
                    var arrangedPlan = plan.ArrangePlan(setting.AssemblyAreaSize, setting.AssemblyAreaOrigin);
                    //
                    var planBlocks = arrangedPlan.SelectMany(assembly => assembly).Select(component => component.Block);
                    DisplayBlockGroup(planBlocks);
                    //
                    var outputDirectory = Path.GetDirectoryName(input) + "/../plan";
                    if (!Directory.Exists(outputDirectory)) Directory.CreateDirectory(outputDirectory);
                    var output = $"{outputDirectory}/{Path.GetFileNameWithoutExtension(input)}.csv";
                    writer.WritePlan(output, arrangedPlan);
                    Console.WriteLine($"{header}\n{input}を\n{output}に保存しました．");
                }
                catch (Exception e)
                {
                    Console.WriteLine($"{header}{input}の組立計画に失敗しました:{e.Message}\n{e.StackTrace}");
                    Console.WriteLine("次のファイルの組立計画を開始します");
                }
            }
            Console.WriteLine("すべての組立計画が終了しました");
        }
    }
}
