API Reference
=============

Complete API reference for the Mini-Arm Python library.

.. toctree::
   :maxdepth: 2

   client

Quick Reference
---------------

**Main Class:**

- :class:`mini_arm.MiniArmClient` - Robot control interface

**Installation:**

.. code-block:: bash

   pip install git+https://github.com/Jshulgach/Mini-Arm.git

**Basic Usage:**

.. code-block:: python

   from mini_arm import MiniArmClient
   
   with MiniArmClient(port='COM3') as client:
       client.home()
       client.set_pose(0.135, 0.0, 0.22)
       client.gripper_open()
