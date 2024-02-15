function Block_Model_Visualization_Formater(filename)
%Creates a file in the same formatation of a assembly plan but without
%making any changes to the block model in the input. This is interesting to
%visualize the block model using the ROS2 simulator.

directory = "../../Block_Printer_System/Models/BlockModel/";
block_model = model_loader(directory + filename + ".txt"); % Load model and adapt it to the Stability Judge format
plan_formatation(block_model,directory + filename + "_visualization" + ".txt",1);
fprintf("\nModel completed. Plan available in %s\n",directory + filename + "_visualization" + ".txt");

end

