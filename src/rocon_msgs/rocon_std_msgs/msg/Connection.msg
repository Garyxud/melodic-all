# A connection can be 1 of 5 types
string PUBLISHER = publisher
string SUBSCRIBER = subscriber
string SERVICE = service
string ACTION_CLIENT = action_client
string ACTION_SERVER = action_server
string INVALID = invalid

# type of connection (from string constants above)
string type

# this is the topic/service name or the action base name
string name 

# the name of the node establishing this connection
string node

# topic, service or action type, e.g. std_msgs/String
string type_msg

# topic, service or action extra type info ( uri for service )
string type_info

# xmlrpc node uri for managing the connection
string xmlrpc_uri