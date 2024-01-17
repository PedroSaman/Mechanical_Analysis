//This value here, for the scale_value used in vp6242 urdf file, corresponds to cm
#define block_size_x 0.38
#define block_size_z 0.3
#define table_x_size 30.0
#define table_y_size 60.0
#define table_z_size 18.0
#define base_x_number_of_blocks 20
#define base_y_number_of_blocks 20
#define base_x_size block_size_x*base_x_number_of_blocks
#define base_y_size block_size_x*base_y_number_of_blocks
#define base_z_size 0.3
#define dispenser_x_size 9*block_size_x
#define dispenser_y_size 20*block_size_x
#define dispenser_z_size 0.2
#define bar_x_size 3.0
#define bar_y_size 62.1
#define bar_z_size 3.0
#define minimum_resolution 0.01
#define table_x_position 25.64
#define gripper_max_opening 1.882
#define WHITE 0
#define RED 1
#define ORANGE 2
#define YELLOW 3
#define GREEN 4
#define BLUE 5
#define BLACK 6
#define SUPPORT 7
#define GRAY 8
#define ONE_ROW 1
#define TWO_ROW 0
#define KNOB_DISTANCE_TO_GRASP 0.055
#define INSERT_DISTANCE 2*block_size_x
#define GRIPPER_POSITION_TO_PICK_CORRECTION 0.2
#define POSITION_TO_PICK_DISTANCE 2.0
#define POSITION_TO_RETREAT 2*block_size_x
#define GRASP_CLAW_STOP_DISTANCE 3*block_size_x
#define BASE_CORRECTION_VALUE -1

//Old pick and place tasks
#define PICK_AND_PLACE 1
#define RETREAT 2
#define RETURN_HOME 3

//Pick and place steps
#define POSITION_TO_PICK 1
#define PRE_OPEN_GRIPPER 2
#define DESCEND_TO_PICK 3
#define CLOSE_GRIPPER 4
#define ASCEND_WITH_BLOCK 5
#define POSITION_TO_PLACE 6
#define DESCEND_TO_PLACE 7
#define OPEN_GRIPPER 8
#define RETREAT_FROM_STRUCTURE 9
#define GO_HOME 10