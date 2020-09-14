#!/usr/bin/env python
# Create a dash server that is also an actionlib client to turtle_actionlib

from __future__ import print_function, division

import os
import sys
import time
import json
import signal
import traceback

import numpy as np

from threading import Lock

import rospy
import rospkg
import actionlib

from actionlib_msgs.msg import GoalStatus
from turtlesim.msg import Pose
from turtle_actionlib.msg import ShapeAction, ShapeGoal

# Plotly, Dash, and Flask
import plotly.graph_objs as go

import dash
import dash_core_components as dcc
import dash_html_components as html

from flask import jsonify


# Helper functions and constants (should ideally be in a utils module)

GOAL_STATUS_TO_TXT = { getattr(GoalStatus, x): x for x in dir(GoalStatus) if x.isupper() }


# The app definition

APP = dash.Dash(
    __name__,
    assets_folder=os.path.join(rospkg.RosPack().get_path('turtlesim_dash_tutorial'), 'dash_assets'),
    external_stylesheets=[
        {
            'href': 'https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css',
            'rel': 'stylesheet',
            'integrity': 'sha384-ggOyR0iXCbMQv3Xipma34MD+dH/1fQ784/j6cY/iJTQUOhcWr7x9JvoRxT2MZw1T',
            'crossorigin': 'anonymous',
        },
    ]
)


class Dashboard(object):
    """
    Create a Flask server to display the UI and a ROS node to send commands to
    the turtlesim
    """

    # Flask
    APP_HOST = '0.0.0.0'
    APP_PORT = 8080
    APP_STATUS_URL = '/ros_api/status'
    APP_STATUS_ENDPOINT = 'ros_status'

    # Actions, Topics, and Services
    # Note that although the values are hard-coded for now, these can be set via
    # service or ROS params if need be (a trivial update)
    TURTLE_SHAPE_ACTION_NAME = 'turtle_shape'
    TURTLE_POSE_TOPIC = '/turtle1/pose'

    # Constants that determine the behaviour of the dashboard
    # Pose is published at ~62 Hz; so we'll see ~30 sec of history. Note that
    # these parameters could be set through ROS parameters or services too!
    POSE_UPDATE_INTERVAL = 5
    POSE_MAX_TIMESTEPS = 2000
    POSE_ATTRIBUTES = ['x', 'y', 'theta', 'linear_velocity', 'angular_velocity']

    # Constants for pertinent output fields
    SERVER_STATUS_OUTPUT_FORMAT = "Shape Server Status: {status}"

    def __init__(self):
        global APP

        # The Flask application
        self._app = APP
        self._flask_server = self._app.server

        # Create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)

        # Initialize the variables that we'll be using to save information
        self._server_status = GoalStatus.LOST
        self._pose_history = np.ones(
            (1+len(Dashboard.POSE_ATTRIBUTES), Dashboard.POSE_MAX_TIMESTEPS)) * np.nan
        self._history_length = 0
        self._pose_history_lock = Lock()

        # Setup the subscribers, action clients, etc.
        self._shape_client = actionlib.SimpleActionClient(Dashboard.TURTLE_SHAPE_ACTION_NAME, ShapeAction)
        self._pose_sub = rospy.Subscriber(Dashboard.TURTLE_POSE_TOPIC, Pose, self._on_pose)

        # Initialize the application
        self._define_app()

    @property
    def pose_history(self):
        return self._pose_history[:, :self._history_length]

    def start(self):
        rospy.loginfo("Connecting to turtle_shape...")
        self._shape_client.wait_for_server()
        rospy.loginfo("...turtle_shape connected.")
        self._app.run_server(host=Dashboard.APP_HOST,
                             port=Dashboard.APP_PORT,
                             debug=False)

    def stop(self, *args, **kwargs):
        # Give some time for rospy to shutdown (cannot use rospy now!)
        print("Shutting down Dash server")
        time.sleep(2)
        sys.exit(0)

    def _define_app(self):
        """
        Define the app layout and callbacks here
        """
        # Define each component of the page

        # First the graph element that will plot the pose and velocity of the
        # robot
        pose_graph_layout = html.Div(dcc.Graph(id='pose', style={ 'width': '100%' }), className='row')

        # Then the section that will update the parameters for the shape that
        # the turtle will trace in the turtle sim
        shape_params_layout = html.Div(
            [
                dcc.Input(id="shape-edges", type='number', placeholder='Num Edges', className='col mx-2'),
                dcc.Input(id="shape-radius", type='number', placeholder='Radius', className='col mx-2'),
                html.Button("Trace Shape", id='trace-button', n_clicks=0, className='btn btn-large btn-primary col-3'),
            ],
            className='row'
        )

        # Then the section that will display the status of the shape server
        server_status_layout = html.Div(
            dcc.Markdown(id='server-status', className='col'),
            className='row my-2'
        )

        # String them all together in a single page
        self._app.layout = html.Div(
            [
                # Hidden button for JS polling
                html.Button(id='refresh-status', n_clicks=0, style={ 'display': 'none' }),

                # The params for tracing the shape
                html.Div(html.H3('Shape Tracing:', className='col'), className='row mt-4'),
                shape_params_layout,
                server_status_layout,

                # The section showing the action status
                html.Div(html.H3('Pose History:', className='col'), className='row my-2'),
                pose_graph_layout,

                # The interval component to update the plots
                dcc.Interval(id='interval-component',
                             n_intervals=0,
                             interval=(Dashboard.POSE_UPDATE_INTERVAL * 1000)),
            ],
            className="container"
        )

        # Define callbacks to update the elements on the page
        self._app.callback(
            dash.dependencies.Output('pose', 'figure'),
            [dash.dependencies.Input('interval-component', 'n_intervals')]
        )(self._define_pose_history_callback())

        # Define a callback to send the goal to the server when the 'Trace'
        # button is clicked. Wait until the client is done executing
        self._app.callback(
            dash.dependencies.Output('trace-button', 'autoFocus'),
            [dash.dependencies.Input('trace-button', 'n_clicks')],
            [dash.dependencies.State('shape-edges', 'value'),
             dash.dependencies.State('shape-radius', 'value')]
        )(self._define_trace_shape_callback())

        # Define a callback to show the status of the server
        self._app.callback(
            dash.dependencies.Output('server-status', 'children'),
            [dash.dependencies.Input('refresh-status', 'n_clicks')]
        )(self._define_server_status_callback())

        # Add the flask API endpoints
        self._flask_server.add_url_rule(
            Dashboard.APP_STATUS_URL,
            Dashboard.APP_STATUS_ENDPOINT,
            self._flask_status_endpoint
        )

    def _define_server_status_callback(self):
        """
        Define a callback to populate the server status display when the status
        refresh button (hidden) is pressed
        """
        def server_status_callback(n_clicks):
            status = GOAL_STATUS_TO_TXT.get(self._server_status)
            return Dashboard.SERVER_STATUS_OUTPUT_FORMAT.format(**locals())

        return server_status_callback

    def _define_trace_shape_callback(self):
        """
        Define a callback that will be invoked every time the 'Trace' button is
        clicked.
        """
        def trace_shape_callback(n_clicks, num_edges, radius):
            # Ignore the 'click' event when the component is created
            if n_clicks is None or n_clicks == 0:
                return False

            # Coerce the input data into formats that we can use
            try:
                num_edges = int(num_edges)
                radius = float(radius)
            except Exception as e:
                rospy.logerr("Error parsing params - {}\n{}".format(e, traceback.format_exc()))
                return False

            # Create the goal and send it to the action server
            goal = ShapeGoal(edges=num_edges, radius=radius)
            self._shape_client.send_goal(goal)
            self._server_status = GoalStatus.ACTIVE

            # Wait for a result
            self._shape_client.wait_for_result()

            # Finally, update the status, log the result, and return true
            self._server_status = self._shape_client.get_state()
            result = self._shape_client.get_result()
            rospy.loginfo("ShapeServer: Interior Angle - {result.interior_angle}, Apothem - {result.apothem}".format(**locals()))

            return True

        return trace_shape_callback

    def _define_pose_history_callback(self):
        """
        Define a callback that will be invoked on every update of the interval
        component. Keep in mind that we return a callback here; not a result
        """
        def pose_history_callback(n_intervals):
            # Get a view into the latest pose history
            pose_history = self.pose_history

            # Create the output graph
            data = [
                go.Scatter(
                    name=attr,
                    x=pose_history[0, :],
                    y=pose_history[idx+1, :],
                    mode='lines+markers'
                )
                for idx, attr in enumerate(Dashboard.POSE_ATTRIBUTES)
            ]
            layout = go.Layout(
                showlegend=True,
                height=500,
                yaxis=dict(
                    fixedrange=True
                ),
                margin=dict(
                    autoexpand=True
                )
            )

            return { 'data': data, 'layout': layout }

        return pose_history_callback

    def _on_pose(self, msg):
        """
        The callback for the position of the turtle on
        :const:`TURTLE_POSE_TOPIC`
        """
        if self._history_length == Dashboard.POSE_MAX_TIMESTEPS:
            self._pose_history[:, :-1] = self._pose_history[:, 1:]
        else:
            self._history_length += 1

        self._pose_history[:, self._history_length-1] = [
            rospy.Time.now().to_time() % 1000,
            msg.x,
            msg.y,
            msg.theta,
            msg.linear_velocity,
            msg.angular_velocity,
        ]

    def _flask_status_endpoint(self):
        return jsonify({
            'server_status': self._server_status,
        })
