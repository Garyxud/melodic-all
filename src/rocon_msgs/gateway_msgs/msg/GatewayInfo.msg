###### Gateway information ######

# Gateway parameters
string name
bool connected
string ip

# Hub parameters
string[] hub_names
string[] hub_uris

bool firewall

###### Public Interface ######

Rule[] public_watchlist
Rule[] public_interface

###### Flipped Interface ######

RemoteRuleWithStatus[] flipped_connections
RemoteRule[] flip_watchlist
RemoteRule[] flipped_in_connections

###### Pulled Interface ######

RemoteRule[] pull_watchlist
RemoteRule[] pulled_connections

