# 
# Provide the conductor graph of all concert clients according to their current state (see
# ConcertClientState.msg for a list of states and always sync with that!).
# Compared to the regular ConcertClients topic, this includes
# all those not in use by this concert, which is useful for introspection and debugging.
#
# It will typically show up in /concert/conductor/graph. Do not use this topic for
# runtime handling (use only for introspection) as it will consume cycles. If no-one is
# listening, this topic is lazy.  
#
concert_msgs/ConcertClient[] pending
concert_msgs/ConcertClient[] available
concert_msgs/ConcertClient[] missing
concert_msgs/ConcertClient[] gone
