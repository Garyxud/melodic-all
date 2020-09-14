#ubnt_airos_tools

Ubiquiti AirOS tools for extracting AP information to ROS.

Tested with Bullet M5 with firmware 6.2.0.

This package provides a node that publishes some basic statistics about the Bullet
itself, its network interfaces and connected stations.

## airos_api

### Parameters

- `string` `username`: Username to use for login (default `ubnt`)
- `string` `password`: Password to use for login (default `ubntubnt`)
- `string` `address`: The base address of the web UI (default `http://192.168.1.1`)
- `string` `frame_id`: The frame_id to be used in the published messages' headers. If `None`, the Bullet's hostname will be read from a status message and used as frame_id. Default is `None`.
- `float` `publish_rate`: Rate at which to query the Bullet and publish the data (default `0.1` Hz).
- `bool` `compute_rates`: The station and interface stats do not normally include rate statistics. If you want the to be computed by this node, set this paramter to True. This means that the first message of each stream will be dropped, because it has no predecessor with which total bytes could be compared. Default is `True`.

### Published topics

- `ubnt_airos_tools/Status` `status`: The status info about the Bullet.
- `ubnt_airos_tools/Interfaces` `interfaces`: Statistics of the Bullet's network interfaces.
- `ubnt_airos_tools/Stations` `stations`: Statistics of the Bullet's connected stations.