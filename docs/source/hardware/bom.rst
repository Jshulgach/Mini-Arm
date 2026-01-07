Bill of Materials
=================

Complete list of parts needed to build a Mini-Arm.

Electronics
-----------

.. list-table::
   :header-rows: 1
   :widths: 40 15 15 30

   * - Component
     - Qty
     - Price (USD)
     - Link
   * - Raspberry Pi Pico 2
     - 1
     - $5
     - `Adafruit <https://www.adafruit.com/product/5544>`_
   * - MG90S Micro Servo
     - 6
     - $18
     - `Amazon <https://amazon.com>`_
   * - SG90 Micro Servo (gripper)
     - 1
     - $3
     - `Amazon <https://amazon.com>`_
   * - PCA9685 Servo Driver
     - 1
     - $6
     - `Amazon <https://amazon.com>`_
   * - USB-C Cable
     - 1
     - $5
     - Any
   * - Jumper Wires (M-F)
     - 20
     - $4
     - `Amazon <https://amazon.com>`_
   * - 5V 3A Power Supply
     - 1
     - $8
     - `Amazon <https://amazon.com>`_

**Electronics Subtotal: ~$49**

Hardware
--------

.. list-table::
   :header-rows: 1
   :widths: 40 15 15 30

   * - Component
     - Qty
     - Price (USD)
     - Notes
   * - M2 x 8mm Screws
     - 20
     - $3
     - Pan head
   * - M2 x 12mm Screws
     - 10
     - $2
     - Pan head
   * - M2 Nuts
     - 30
     - $2
     - 
   * - M3 x 10mm Screws
     - 10
     - $2
     - Pan head
   * - M3 Nuts
     - 10
     - $2
     - 
   * - 608 Bearings (8x22x7mm)
     - 4
     - $5
     - For smooth rotation
   * - Rubber Feet
     - 4
     - $2
     - Anti-slip base

**Hardware Subtotal: ~$18**

3D Printed Parts
----------------

All parts can be printed on a standard FDM printer.

.. list-table::
   :header-rows: 1
   :widths: 50 15 20 15

   * - Part
     - Qty
     - Material
     - Time
   * - Base
     - 1
     - PLA/PETG
     - 3h
   * - Shoulder
     - 1
     - PLA/PETG
     - 2h
   * - Upper Arm
     - 1
     - PLA/PETG
     - 2h
   * - Forearm
     - 1
     - PLA/PETG
     - 1.5h
   * - Wrist
     - 1
     - PLA/PETG
     - 1h
   * - Gripper Base
     - 1
     - PLA/PETG
     - 0.5h
   * - Gripper Fingers
     - 2
     - PLA/PETG
     - 0.5h
   * - Servo Mounts
     - 6
     - PLA/PETG
     - 1h
   * - Electronics Mount
     - 1
     - PLA/PETG
     - 1h

**3D Printing: ~$15** (material cost) / **~12 hours** print time

Print settings:
- Layer height: 0.2mm
- Infill: 20%
- Supports: Yes (for some parts)
- Material: PLA recommended, PETG for durability

Download STL files from ``assets/stl/`` or `Printables <https://www.printables.com>`_.

Optional Upgrades
-----------------

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Upgrade
     - Price
     - Benefit
   * - MG996R Servos
     - $30
     - Higher torque, larger payload
   * - Slip Ring
     - $8
     - Continuous base rotation
   * - Camera Mount + USB Camera
     - $15
     - Vision applications
   * - Force Sensor (gripper)
     - $10
     - Grasp force feedback
   * - Limit Switches
     - $5
     - Precise homing

Total Cost Summary
------------------

.. list-table::
   :widths: 50 50

   * - **Basic Build**
     - **~$82**
   * - Electronics
     - $49
   * - Hardware
     - $18
   * - 3D Printing (material)
     - $15
   * - 
     - 
   * - **With All Upgrades**
     - **~$150**

Next Steps
----------

- :doc:`3d_printing` - Print settings and tips
- :doc:`assembly` - Step-by-step build guide
