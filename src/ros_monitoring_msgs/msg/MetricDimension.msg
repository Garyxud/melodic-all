###################################
# Metric Dimensions provide an additional way to group, sort, and filter metrics.
#
# For example, you may have a large number of robots running in different factories. They are all publishing a metric
# that indicates how long it takes them to perform a specific task. If we're just looking at all the metrics coming from
# all the robots then we lose the ability to figure out things like if robots in one factory take longer to perform a
# task than in another factory. Dimensions allow us to solve this by include a dimension name "factory" with the factory
# name the robot is running in with each metric it publishes. Then we could look at the metrics for all robots or filter
# down to just robots in a specific factory.
#
# Additionally, if we wanted to A/B test software versions we could add another dimension named "software_version" that
# has a value set to the version of software the metric is produced from. Then we could group by the version of software
# as well as which factory the robot is in.
#
###################################

# The name of the Metric Dimension
string name

# The value for the dimension
string value
