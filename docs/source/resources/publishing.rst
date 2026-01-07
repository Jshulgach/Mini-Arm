Publishing to PyPI
==================

This guide explains how to publish the ``mini-arm`` package to PyPI.

Prerequisites
-------------

1. Create accounts on `PyPI <https://pypi.org>`_ and `TestPyPI <https://test.pypi.org>`_
2. Install build tools:

.. code-block:: bash

   pip install build twine

Build the Package
-----------------

From the repository root:

.. code-block:: bash

   # Clean previous builds
   rm -rf dist/ build/ src/*.egg-info

   # Build source and wheel distributions
   python -m build

This creates:

- ``dist/mini_arm-X.Y.Z.tar.gz`` (source distribution)
- ``dist/mini_arm-X.Y.Z-py3-none-any.whl`` (wheel)

Test on TestPyPI
----------------

Always test on TestPyPI first:

.. code-block:: bash

   # Upload to TestPyPI
   python -m twine upload --repository testpypi dist/*

   # Test installation
   pip install --index-url https://test.pypi.org/simple/ mini-arm

Publish to PyPI
---------------

Once verified on TestPyPI:

.. code-block:: bash

   # Upload to PyPI
   python -m twine upload dist/*

   # Verify installation
   pip install mini-arm

Using API Tokens (Recommended)
------------------------------

Instead of username/password, use API tokens:

1. Go to PyPI → Account Settings → API tokens
2. Create a token scoped to the ``mini-arm`` project
3. Create ``~/.pypirc``:

.. code-block:: ini

   [pypi]
   username = __token__
   password = pypi-YOUR-TOKEN-HERE

   [testpypi]
   username = __token__
   password = pypi-YOUR-TESTPYPI-TOKEN-HERE

Automated Publishing with GitHub Actions
----------------------------------------

Add this workflow to ``.github/workflows/publish.yml``:

.. code-block:: yaml

   name: Publish to PyPI

   on:
     release:
       types: [published]

   jobs:
     publish:
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v4
         
         - name: Set up Python
           uses: actions/setup-python@v5
           with:
             python-version: '3.11'
         
         - name: Install build tools
           run: pip install build twine
         
         - name: Build package
           run: python -m build
         
         - name: Publish to PyPI
           env:
             TWINE_USERNAME: __token__
             TWINE_PASSWORD: ${{ secrets.PYPI_API_TOKEN }}
           run: python -m twine upload dist/*

Then add ``PYPI_API_TOKEN`` to your repository secrets.

Version Checklist
-----------------

Before publishing a new version:

1. ☐ Update ``src/mini_arm/version.py``
2. ☐ Update ``pyproject.toml`` version  
3. ☐ Update ROS2 ``package.xml`` versions
4. ☐ Update ``CHANGELOG.md``
5. ☐ Run tests: ``pytest tests/ -v``
6. ☐ Build docs: ``cd docs && make html``
7. ☐ Commit and tag: ``git tag -a vX.Y.Z``
8. ☐ Push: ``git push origin main --tags``
9. ☐ Create GitHub release
10. ☐ Publish to PyPI

Verifying Installation
----------------------

After publishing, verify the package works:

.. code-block:: python

   import mini_arm
   print(mini_arm.__version__)

   from mini_arm import MiniArmClient
   # Test basic functionality
