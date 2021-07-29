===========
Development
===========

Contributing
============

We welcome contributions to the repository. To find things we're currently looking for help with, `check out the open issues <https://github.com/jtebert/kilosim/issues?utf8=✓&q=is%3Aissue+is%3Aopen+>`_.

If you discover a bug or have an idea, for an improvement, `open an issue <https://github.com/jtebert/kilosim/issues/new>`_.

If you add something you think would be useful to others, submit a pull request. All code should be `fully documented <http://www.doxygen.nl/manual/docblocks.html>`_. You can check your documentation by :ref:`building the documentation locally <Build Documentation>` Code reviews will be required for all pull requests, but we currently do not perform unit testing.

See a `full list of the contributors and their contributions here <contributors>`_.

Build Documentation
===================

Build the HTML documentation: (Requires Doxygen and Sphinx to be installed.)

.. code-block:: console

    $ make docs

This will automatically build the documentation and save the results to ``build/docs/sphinx``. You can view it by opening ``build/docs/sphinx/index.html``. The latest full documentation is automatically generated and published `on Read The Docs <https://kilosim.readthedocs.io/en/latest/>`_.