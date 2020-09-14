.. _creating-section-label:

Creating Bubble Icons
=====================

Templates
---------

Two gimp templates are stored on a file server:

.. code-block:: bash

   roscd rocon_bubble_icons
   make templates

Resizing Your Icon
------------------

Once you've found your icon that you want to embed in the bubble, open it in gimp and
resize it to

- ~50x50 pixels (low-res template)
- ~250x250 pixels (high-res template)

Embedding in the Bubble
-----------------------

Open the bubble template you wish to use and make sure you have the ``Layers, Channels, Paths`` window open.

- Select the ``Embedded Icon`` layer
- Use the ``Rectangle Select Tool`` to select the entire turtlebot and then ``cut`` it from the layer.
- Go back to your original scaled down icon, use the same tool to select it and ``copy`` it.
- Go back to the template icon, ``paste into`` the empty embedded icon layer.

Optional:

- Move this layer up and down for best effect.

Saving:

- *Precise*: Use the ``File->Save as`` option
- *Saucy++* : Use the ``File->Export`` option
- Save it as ``png`` type
- When flattening the image, a compression level of 2 seems to work well.

