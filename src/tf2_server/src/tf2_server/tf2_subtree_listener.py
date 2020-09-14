import rospy
from rospy import TransportException
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, InvalidArgumentException, TimeoutException, Buffer
from tf2_server.srv import RequestTransformStream, RequestTransformStreamRequest, RequestTransformStreamResponse


class TransformSubtreeListener(TransformListener):
    def __init__(self, subtree, buffer, queue_size=None, buff_size=65536, tcp_nodelay=False,
                 max_server_wait=rospy.Duration(-1), server_name="tf2_server"):
        assert isinstance(subtree, RequestTransformStreamRequest)
        assert isinstance(buffer, Buffer)

        TransformListener.__init__(self, buffer, queue_size, buff_size, tcp_nodelay)
        self.unregister()  # immediately cancel subscription to /tf and /tf_static

        self._queue_size = queue_size
        self._buff_size = buff_size
        self._tcp_nodelay = tcp_nodelay
        self._topics = None

        service_name = rospy.resolve_name("~request_transform_stream", rospy.resolve_name(server_name))
        self._requestTransformStream = rospy.ServiceProxy(service_name, RequestTransformStream)
        rospy.loginfo("Waiting for service " + service_name, logger_name="tf2_subtree_listener")

        try:
            self._requestTransformStream.wait_for_service(max_server_wait.to_sec())
        except rospy.ROSException, e:
            raise TimeoutException(str(e))

        rospy.loginfo("Service " + service_name + " is available now", logger_name="tf2_subtree_listener")

        self.update_subtree(subtree)

    def unregister(self):
        if self.tf_sub is not None and self.tf_static_sub is not None:
            TransformListener.unregister(self)
            self.tf_sub = None
            self.tf_static_sub = None

    def update_subtree(self, subtree):
        rospy.loginfo("Requesting topic names for transform subtree", logger_name="tf2_subtree_listener")
        try:
            topics = self._requestTransformStream.call(subtree)
            assert isinstance(topics, RequestTransformStreamResponse)
        except rospy.ServiceException, e:
            raise InvalidArgumentException(str(e))

        if self._topics is None or topics != self._topics:
            self._topics = topics
            self.unregister()

            self.buffer.clear()

            self.tf_sub = rospy.Subscriber(
                topics.topic_name, TFMessage, self.callback, queue_size=self._queue_size,
                buff_size=self._buff_size, tcp_nodelay=self._tcp_nodelay)
            self.tf_static_sub = rospy.Subscriber(
                topics.static_topic_name, TFMessage, self.static_callback, queue_size=self._queue_size,
                buff_size=self._buff_size, tcp_nodelay=self._tcp_nodelay)

            rospy.loginfo("Created transform subtree listener with /tf:=%s and /tf_static:=%s" % (
                topics.topic_name, topics.static_topic_name), logger_name="tf2_subtree_listener")
