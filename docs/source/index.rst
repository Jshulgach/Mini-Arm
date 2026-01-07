Mini-Arm Documentation
======================

.. image:: _static/mini-arm-banner.jpg
   :align: center
   :alt: Mini-Arm Robot

|

**Mini-Arm** is a miniature 6-DOF 3D-printed robot arm with an onboard inverse kinematics solver,
CircuitPython firmware, and full ROS2 integration—all for under $100.

.. grid:: 2

    .. grid-item-card:: 🚀 Getting Started
        :link: getting_started/installation
        :link-type: doc

        Install the software and get your Mini-Arm up and running in minutes.

    .. grid-item-card:: 🔧 Hardware Guide
        :link: hardware/assembly
        :link-type: doc

        Build your own Mini-Arm with our step-by-step assembly instructions.

    .. grid-item-card:: 📖 Tutorials
        :link: tutorials/index
        :link-type: doc

        Learn through hands-on examples from basic control to AI integration.

    .. grid-item-card:: 📚 API Reference
        :link: api/modules
        :link-type: doc

        Complete Python API documentation for the MiniArm client library.

Key Features
------------

- **Raspberry Pi Pico 2** - Runs CircuitPython firmware with onboard IK solver
- **Under $100** - Affordable robotics for education and research
- **95% 3D Printable** - Only fasteners and servos required
- **ROS2 Integration** - Full MoveIt2 support for motion planning
- **Python API** - Simple, intuitive control from any Python environment
- **Lightweight** - Less than 1 lb (~0.3 kg)

.. toctree::
   :maxdepth: 1
   :caption: Getting Started
   :hidden:

   getting_started/installation
   getting_started/quickstart
   getting_started/configuration

.. toctree::
   :maxdepth: 1
   :caption: Hardware
   :hidden:

   hardware/bom
   hardware/3d_printing
   hardware/assembly
   hardware/wiring

.. toctree::
   :maxdepth: 1
   :caption: Tutorials
   :hidden:

   tutorials/index
   tutorials/basic_control
   tutorials/trajectories
   tutorials/xbox_teleop
   tutorials/face_tracking
   tutorials/ai_integration

.. toctree::
   :maxdepth: 1
   :caption: API Reference
   :hidden:

   api/modules

.. toctree::
   :maxdepth: 1
   :caption: ROS2 Integration
   :hidden:

   ros2/setup
   ros2/visualization
   ros2/moveit

.. toctree::
   :maxdepth: 1
   :caption: Resources
   :hidden:

   resources/faqs
   resources/troubleshooting
   resources/contributing
   resources/publishing
   resources/changelog

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
