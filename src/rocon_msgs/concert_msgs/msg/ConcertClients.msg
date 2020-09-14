
# Clients that have been invited to this concert and can be used
concert_msgs/ConcertClient[] clients

# Clients that have disappeared from the gateway network without formally leaving. 
# Usually this will mean they have dropped off the wireless
concert_msgs/ConcertClient[] missing_clients

# Clients that are on the gateway network, but haven't been invited by this concert yet
# We do not have any implementation that can invite except by a conductor's auto_invite yet.
concert_msgs/ConcertClient[] pending_clients
