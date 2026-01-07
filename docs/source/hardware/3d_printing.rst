3D Printing Guide
=================

Detailed instructions for printing Mini-Arm components.

Recommended Settings
--------------------

**General Settings:**

.. list-table::
   :widths: 30 70

   * - Layer Height
     - 0.2mm (0.16mm for fine details)
   * - Nozzle
     - 0.4mm standard
   * - Infill
     - 20% (gyroid or grid)
   * - Wall Count
     - 3-4 perimeters
   * - Top/Bottom Layers
     - 4 layers
   * - Support
     - Yes, where needed
   * - Bed Adhesion
     - Brim (5mm)

**Material Recommendations:**

- **PLA** - Easy to print, sufficient for most uses
- **PETG** - Better durability, slight flexibility
- **ABS** - Heat resistant, requires enclosure

Part-by-Part Settings
---------------------

Base
^^^^

.. code-block:: text

    Orientation: Flat (large face down)
    Supports: No
    Infill: 30% (structural)
    Notes: Use brim for adhesion

Shoulder & Upper Arm
^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    Orientation: On side (bearing holes vertical)
    Supports: Yes (touching buildplate only)
    Infill: 20%
    Notes: Critical for bearing fit

Forearm & Wrist
^^^^^^^^^^^^^^^

.. code-block:: text

    Orientation: Servo mount facing up
    Supports: Yes
    Infill: 20%
    Notes: Check servo horn clearance

Gripper
^^^^^^^

.. code-block:: text

    Orientation: Flat
    Supports: Minimal
    Infill: 15%
    Notes: Print fingers separately

Post-Processing
---------------

1. **Remove Supports** - Use flush cutters, then sand
2. **Test Fit Bearings** - May need light sanding
3. **Thread Brass Inserts** (optional) - Use soldering iron
4. **Chamfer Edges** - Utility knife for cleaner look

Troubleshooting
---------------

**Parts don't fit together:**

- Check printer calibration (print a calibration cube)
- Adjust horizontal expansion in slicer (-0.1 to -0.2mm)
- Light sanding on tight fits

**Weak/brittle parts:**

- Increase wall count to 4+
- Increase infill to 30%
- Check for under-extrusion

**Warping:**

- Use heated bed (60°C for PLA)
- Ensure proper first layer adhesion
- Use enclosure for ABS

File Downloads
--------------

STL files are available in the repository:

.. code-block:: bash

    Mini-Arm/assets/stl/
    ├── base.stl
    ├── shoulder.stl
    ├── upper_arm.stl
    ├── forearm.stl
    ├── wrist.stl
    ├── gripper_base.stl
    ├── gripper_finger_left.stl
    ├── gripper_finger_right.stl
    └── electronics_mount.stl

Or download from `Printables <https://www.printables.com>`_ (link TBD).

STEP files for modifications are in ``assets/cad/``.

Next Steps
----------

- :doc:`assembly` - Build your Mini-Arm
- :doc:`wiring` - Electronics installation
