========================
Use Kilosim in a Project
========================

.. contents:: On this page
  :local:

Beyond testing your dependencies and installation, you probably want to use Kilosim as part of another project.

One way to do this in your project is to use that project's Cmake file to point to an installation of Kilosim elsewhere on your system. However, this is non-portable (you will have to install Kilosim separately again to use it on a different computer), and risks having an incorrect or incompatible version installed.

A better approach is to reference a version of the Kilosim repository within your project as a `submodule <https://git-scm.com/book/en/v2/Git-Tools-Submodules>`_. This will let your project keep track of its own correct version Kilosim, without having to install it separately or keep a copy of it within the project repository.

Set up Kilosim as a submodule
=============================

When you first create your project using Kilosim, you'll need to do a one-time setup to initialize the submodule.

Start by creating a folder for your project, and initializing it as a git repository with ``git init``.

Now we can add Kilosim as a submodule.

.. code-block:: bash

    $ mkdir submodules
    $ cd submodules
    $ git submodule add git@github.com:jtebert/kilosim.git

This will pull the latest version of Kilosim to the submodules folder (which will always be the latest release version). If you need to use an older version, you can go to the `releases page <https://github.com/jtebert/kilosim/releases>`_ to get the tag name of the version you want. For example, ``v0.4.1``. You can then check out that specific version of Kilosim.

.. code-block:: bash

    $ cd kilosim
    $ git checkout <release_tag>

Note that the version of Kilosim you are using will be preserved whenever your project repository is cloned in the future. If you want to upgrade or switch to a different version in the future, see :ref:`Updating Kilosim`.

Set up project CMake
====================

To use this submodule, you will need to include it in your project's ``CMakeLists.txt`` file. For this example, we'll call the project ``kilosim_demo``. Your CMakeLists.txt file will look something like this:

.. code-block:: cmake

    cmake_minimum_required (VERSION 3.9)

    project(kilosim_demo LANGUAGES CXX C)

    # Identify the location of the kilosim library
    add_subdirectory(submodules/kilosim)

    # Directory containing header files
    include_directories(api)

    # Be sure to list all source files
    add_executable(kilosim_demo
    src/MyBot.cpp
    src/main.cpp
    )

    # Link the kilosim library
    target_link_libraries(kilosim_demo PRIVATE kilosim)

You will also need to add whatever source files you're using here. In this example, that's just ``MyBot.cpp`` and ``main.cpp``.

Compile project
===============

Now that your project is set up, you can compile it for the first time.

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../ ..
    $ make

This will compile your project within the ``build`` folder. If you want to install it, you can do so with ``make install``. This will place the executable in the ``bin`` directory (in the root of your project folder.

You should also make sure that the ``build`` and ``bin`` directories are in your ``.gitignore`` file, so that you don't accidentally add them to your project repository.

Clone project
=============

When you (or someone else!) wants to clone the project from its source (e.g., GitHub), you need to make sure it's cloned such that it includes the Kilosim submodule. You can do this with:

.. code-block:: bash

    $ git clone --recurse-submodules <project_git_source>

If you forget this, you can clone the submodules later with:

.. code-block:: bash

    $ git submodule update -- init --recursive

I recommend adding these instructions to your project's README, so that you remember the right way to clone the project in the future!

Updating Kilosim
================

To update Kilosim to the latest version, you can simply pull the latest version from Github.

.. code-block:: bash

    $ cd submodules/kilosim
    $ git pull

If you want to switch to a specific version, you can select it by the release tag:

.. code-block:: bash

    $ cd submdodules/kilosim
    $ git checkout <release_tag>