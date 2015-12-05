//This is the base file that will handle all the relevant part of processing
#include <cmt_tracker/Face.h>
#include <cmt_tracker/Faces.h>

class cmt_tracker
{
/**
The requirements of the this class: 
1. This the function spawns the CMT instances. 
  1.1 This is the fucntion that initalized. 
  1.2 This is the one that publishes the location of the tracker results to the system. 
  1.3 This is the fucntion that makes the decision on what to track and what not to track. 

2. This is the fucntion that holds parameters for the tracker configuration and the one that would be modified when changes are made the plugin. 
  2.1 When a person changes the configuration in the parameter server; this fucntion must change the way it performs it's tasks. 
  2.2 When the CMT is initialized it's  must get the parameters from the systme so that it would perfrom in house calculation to maintain decent results. 
  2.3 The tracking results are maintained here and updated correspondingly. 

3. THIS FUNCTION MUST BE THREADED. 

How i should proceed is this; 

1st. Takeout everything from the tracker_plugin and make it in the cmt_tracker_node
  This step includes  1. Make the CMT_Tracker Listen to parameter settings and start processing the elements. 
                      2. Make a detailed description of the output to rely to the other system. 
                      3. Evaluate the performance of the system. 
*/

}