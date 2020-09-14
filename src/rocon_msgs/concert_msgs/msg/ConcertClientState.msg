# States of concert clients, these should always be synced with what is provided in ConductorGraph.msg
# (for completeness, but also because I do some funny magic to extra the possible list of states from there). 

# pending introspection, the client has been detected, but hasn't signalled that it wants to work with us yet
string PENDING   = pending
# the ip is invalid, unreachable or some other problem meaning we have to absolutely ignore it
string BAD       = bad
# is available for use inside this concert
string AVAILABLE = available
# has lost connection (wireless)
string MISSING   = missing
# has left the concert
string GONE      = gone
