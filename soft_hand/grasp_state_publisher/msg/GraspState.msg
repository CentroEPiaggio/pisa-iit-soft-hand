# This is a message that holds data to describe the state of a grasp by a hand
# The decision is based in a Naive Bayesian Classifier

Header header

# hand name
string name

# decision based on confidences
bool grasp
bool slip

# confidences, in case they are required, it is the output of a pdf
float64 grasp_confidence
float64 no_grasp_confidence

float64 slip_confidence
float64 no_slip_confidence
