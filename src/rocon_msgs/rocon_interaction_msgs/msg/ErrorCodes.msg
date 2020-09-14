# Error types for interactions

int8 SUCCESS = 0

# Interaction errors
int8 INTERACTION_UNAVAILABLE = 20    # The requested role-app pair is not available
int8 INTERACTION_QUOTA_REACHED = 21  # Maximum number or role-app connections already reached 

# Pairing errors
int8 PAIRING_UNAVAILABLE          = 30
int8 START_PAIRING_FAILED         = 31
int8 ALREADY_PAIRING              = 32
int8 REQUIRED_RAPP_IS_NOT_RUNNING = 33
int8 DIFFERENT_RAPP_IS_RUNNING    = 34
int8 NOT_PAIRING                  = 35
int8 STOP_PAIRING_FAILED          = 36

string MSG_INTERACTION_UNAVAILABLE                  = "This interaction is not available for use."
string MSG_PAIRING_UNAVAILABLE                      = "The specified pairing does not exist."
string MSG_INTERACTION_QUOTA_REACHED                = "More connections of this type not permitted."
string MSG_REQUIRED_RAPP_IS_NOT_RUNNING             = "No rapp is running and this pairing interaction requires one to be."
string MSG_DIFFERENT_RAPP_IS_RUNNING                = "A different rapp to that required is already running."
string MSG_START_PAIRING_FAILED                     = "Failed to start the pairing (rapp)."
string MSG_STOP_PAIRING_FAILED                      = "Failed to stop a pairing (rapp)"
string MSG_ALREADY_PAIRING                          = "Already pairing, cannot start another pairing."
string MSG_NOT_PAIRING                              = "This interaction manager is not managing paired interactions."