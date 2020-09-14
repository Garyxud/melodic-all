# Welcome to Plotjuggler

PlotJuggler is an application to plot logged data, in particular timeseries.

The design focus on simplicty of use rather than completeness. It you want to
submit a issue report or a suggestion, don't hesitate to visit the 
[official webpage](https://github.com/facontidavide/PlotJuggler).

![PlotJuggler](images/PlotJuggler.gif)

## Architecture

The functionalities of the application can be extended using __plugins__. 
There are three types of plugins:

- __DataLoaders__: they are used to load static data from log files.
- __DataStreamers__: used to load and plot data that is continously streamed to the user.
- __StatePublishers__: provide a way to publish data that was previously loaded.

__Note__: [ROS](www.ros.org) funtionalities are implemented exclusively 
through plugins. The core of PlotJuggler does not (and will not) depend on ROS.





