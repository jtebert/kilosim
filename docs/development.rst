===========
Development
===========

.. contents:: On this page
  :local:

Contributing
============

We welcome contributions to the repository. To find things we're currently looking for help with, `check out the open issues <https://github.com/jtebert/kilosim/issues?utf8=✓&q=is%3Aissue+is%3Aopen+>`_.

If you discover a bug or have an idea, for an improvement, `open an issue <https://github.com/jtebert/kilosim/issues/new>`_.

If you add something you think would be useful to others, submit a pull request. All code should be `fully documented <http://www.doxygen.nl/manual/docblocks.html>`_. If a function must be public to be used by another class, but is not required for the user API, the documentation should be surrounded by ``\internal`` & ``\endinternal`` tags so that it will not appear in the public API documentation. You can check your documentation by :ref:`building the documentation locally <Building documentation (locally)>`

Code reviews will be required for all pull requests, but we currently do not perform unit testing.

See a `full list of the contributors and their contributions here <https://github.com/jtebert/kilosim/blob/master/CONTRIBUTORS.md>`_.

Testing & validation
====================

Although unit tests are not required, there will soon be a set of examples that should produce a valid simulation with outputs that will be described here.

Release checklist
=================

These requirements must be met before a new release is made. A new release will be made when a pull request is merged to the master branch.

- Verify tests and examples work. (In the future, this will be autmated with continuous integration on GitHub.)
- Check that all documentation is updated.
- Update version number. Right now, it has to be manually update in multiple places (which will be consolidated in the future):

  - CMakeLists.txt ``VERSION`` in ``project``
  - conf.py ``release``
  - Doxyfile.in ``PROJECT_NUMBER``
- Update the citation information for Zenoda/bibtex. (link, release date, etc.)
- Update changelog: move "Unreleased" to new version.
- Add any new contributors to ``CONTRIBUTORS.md``.
- Submit a pull request to merge the changes to the master branch.
- Create release on Github. (This will automatically create a new Stable and version-numbered documentation version on Read The Docs.)


Building documentation (locally)
================================

Build the HTML documentation: (Requires Doxygen and Sphinx to be installed.)

.. code-block:: console

    $ make docs

This will automatically build the documentation and save the results to ``build/docs/sphinx``. For anything undocumented, you will see a warning printed in the console. You can view the compiled documentation by opening ``build/docs/sphinx/index.html``.

The latest full documentation is automatically generated and published `on Read The Docs <https://kilosim.readthedocs.io/en/latest/>`_.

Documentation will be automatically generated when a pull request is merged to the master branch (as "latest"), and new releases will produce version-tagged documentation.

Debugging and profiling tips
============================

These are some helpful tips for local development.

Performance profiling
---------------------

Do this to figure out what functions are taking up the most time in the code.

To profile with ``gprof`` you need to run a different ``cmake`` command (from the build folder):

.. code-block:: console

    $ cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-pg -DCMAKE_INSTALL_PREFIX=../ ..

This will generate a file called ``gmon.out``. You can create an analysis with:

.. code-block:: console

    $ gprof gridbots_decisions gmon.out  > analysis.txt


Memory profiling
----------------

You can use ``valgrind`` to profile memory usage. This is also useful for tracking down the source of runtime errors (like segfaults).

Instead of running your code as (for example) ``./bin/kilosim_demo``, run it as:

.. code-block:: console

    $ valgrind --tools=memcheck --leak-chck=yes ./kilosim_demo


Additional notes
================

This documentation was built to Sphinx/Read The Docs from the existing Doxygen documentation following `this guide <https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/>`_.