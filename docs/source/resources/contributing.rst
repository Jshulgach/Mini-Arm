Contributing
============

Thank you for your interest in contributing to Mini-Arm!

Ways to Contribute
------------------

- 🐛 **Report bugs** - Open an issue on GitHub
- 💡 **Suggest features** - Start a discussion
- 📖 **Improve documentation** - Fix typos, add examples
- 🔧 **Submit code** - Bug fixes, new features
- 🎨 **Share your builds** - Post photos, videos, modifications

Development Setup
-----------------

1. Fork the repository on GitHub

2. Clone your fork:

.. code-block:: bash

    git clone https://github.com/YOUR_USERNAME/Mini-Arm.git
    cd Mini-Arm

3. Create a virtual environment:

.. code-block:: bash

    python -m venv venv
    source venv/bin/activate  # Linux/Mac
    # or: venv\Scripts\activate  # Windows

4. Install in development mode:

.. code-block:: bash

    pip install -e ".[dev]"

5. Create a branch for your changes:

.. code-block:: bash

    git checkout -b feature/my-new-feature

Code Style
----------

We use:

- **Black** for code formatting
- **isort** for import sorting
- **flake8** for linting
- **mypy** for type checking

Run all checks:

.. code-block:: bash

    black .
    isort .
    flake8
    mypy mini_arm

Or use pre-commit:

.. code-block:: bash

    pip install pre-commit
    pre-commit install

Testing
-------

Run tests:

.. code-block:: bash

    pytest tests/

Run with coverage:

.. code-block:: bash

    pytest --cov=mini_arm tests/

Submitting Changes
------------------

1. Commit your changes:

.. code-block:: bash

    git add .
    git commit -m "Add feature: description"

2. Push to your fork:

.. code-block:: bash

    git push origin feature/my-new-feature

3. Open a Pull Request on GitHub

4. Wait for review and address feedback

PR Guidelines
-------------

- Keep PRs focused on a single change
- Include tests for new functionality
- Update documentation as needed
- Follow existing code style
- Write clear commit messages

Documentation
-------------

Build docs locally:

.. code-block:: bash

    cd docs
    pip install -r requirements.txt
    make html

View at ``docs/build/html/index.html``

Hardware Contributions
----------------------

If you've designed:

- Modified parts (stronger, lighter, etc.)
- New end-effectors
- Mounting solutions
- Sensor integrations

Share by:

1. Adding STL/STEP files to ``assets/community/``
2. Documenting in ``docs/source/hardware/community.rst``
3. Opening a PR

Code of Conduct
---------------

- Be respectful and inclusive
- Welcome newcomers
- Focus on constructive feedback
- No harassment or discrimination

Questions?
----------

- GitHub Discussions: `Mini-Arm Discussions <https://github.com/Jshulgach/Mini-Arm/discussions>`_
- Open an issue for bugs/features

Thank you for contributing! 🤖
