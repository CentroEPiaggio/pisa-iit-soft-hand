# This is a message that holds data to describe the state of a set of hands and whether they are grasping something or not. 
#
# The state of each hand is defined by:
#  * the grasp of the hand (true or false),
#  * the slip of the grasp (true, the object is moving w.r.t. the hand, probably falling, or false, the grasp is stable).
#
# Each hand is uniquely identified by its name
# The header specifies the time at which the grasps were estimated. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the grasp state. 
# The fields are optional, you can leave slip[] emtpy if you don't know how to estimate that, for instance.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the hand name with the correct
# states.

Header header

string[] name
bool[] grasp
bool[] slip