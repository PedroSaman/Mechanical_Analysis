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
                Console.WriteLine("Could not open configuration file. Creating a default configuration file.");
                try
                {
                    using (var fileWriter = new FileStream(settingFilename, FileMode.Create))
                    {
                        using (var jsonWriter = JsonReaderWriterFactory.CreateJsonWriter(fileWriter, Encoding.UTF8, true, true, "  "))
                        {
                            serializer.WriteObject(jsonWriter, PlannerSetting.Default);
                        }
                    }
                    Console.WriteLine("Prescribed configuration files have been created.");
                }
                catch (Exception e)
                {
                    Console.WriteLine($"Could not create the configuration file:{e.Message}");
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
                    Console.WriteLine($"A block of size {size} color {color} requires {count} pieces.");
                }
                else if (g.Key.BlockAttributeCollection.Exists<Support>())
                {
                    Console.WriteLine($"Support blocks of size {size} require {count} blocks.");
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
                Console.WriteLine("Please pass one or more files as input to this program.");
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
                    Console.WriteLine($"Loading the file {header}{input}...");
                    var components = reader.ReadComponentsFrom(input);
                    Console.WriteLine($"{header}{input} loaded. Number of blocks:{components.Count()}");
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
                    Console.WriteLine($"{header}\n{input}file saved as \n{output}.");
                }
                catch (Exception e)
                {
                    Console.WriteLine($"{header}{input} file assembly plan failed: {e.Message}\n{e.StackTrace}");
                    Console.WriteLine("Start assembly planning for the following files");
                }
            }
            Console.WriteLine("All assembly plans have been completed.");
        }
    }
}
