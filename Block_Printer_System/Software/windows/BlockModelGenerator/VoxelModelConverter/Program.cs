using System;
using System.Linq;
using System.IO;
using Core;
using Core.Serialization;
using Core.Blockalization;

namespace VoxelModelConverter
{
    class Program
    {
        static void Main(string[] args)
        {
            if (!args.Any())
            {
                Console.WriteLine("このプログラムには1つ以上のファイルを入力として渡してください");
                return;
            }
            foreach (var (input, index) in args.WithIndex())
            {
                var header = $"[{index + 1}/{args.Length}] ";
                try
                {
                    Console.WriteLine($"{header}{input}を読み込みます..");
                    var voxels = VoxelCollectionReader.LoadVoxelModel(input);
                    Console.WriteLine($"{header}{input}を読み込みました．ボクセル数:{voxels.Count()}");
                    var blocks = VoxelConverter.CreateBlockModel(voxels);
                    var directory = Path.GetDirectoryName(input) + "/../BlockModel";
                    if (!Directory.Exists(directory)) Directory.CreateDirectory(directory);
                    var output = directory + "/" + Path.GetFileNameWithoutExtension(input) + ".txt";
                    BlockModelWriter.WriteBlockModel(output, blocks);
                    Console.WriteLine($"{header}\n{input}を\n{output}に保存しました．");
                }
                catch (Exception e)
                {
                    Console.WriteLine($"{header}{input}のブロックモデル変換に失敗しました:{e.Message}");
                    Console.WriteLine("次のファイルのブロックモデル変換を開始します");
                }
            }
            Console.WriteLine("すべてのブロックモデル変換が終了しました");
        }
    }
}
