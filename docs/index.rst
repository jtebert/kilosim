.. Kilosim documentation master file, created by
   sphinx-quickstart on Wed Jul 28 16:25:54 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Kilosim Documentation
=====================

.. toctree::
   :maxdepth: 2
   :titlesonly:

   self
   getting_started/index
   reference/index
   development

About
=====

Kilosim is a fast, lightweight, pseudo-physical* C++ simulator for Kilobot robots. Notable benefits include:

- Pseudo-physical model means fast simulation for high-throughput
- Easily re-use code written for Kilobots, using the same Kilolib API
- Includes support for ambient light sensing
- Included ``Logger`` to easily to save experiment parameters in log continuous state data
- Cross-platform ``Viewer`` for debugging and recording simulations
- Easy configuration with JSON files to run multiple trials and varied experiments
- *[In progress]* Parallelization with OpenMP

\* *Pseudo-physical means that it is spatial and handles issues like collisions in a functional but hand-wavy manner. We make no attempt to accurately model true physical interactions. If you want to see what this means, run an example simulation with the Viewer.*

Note that this project is currently in a pre-release state and may still contain bugs, incomplete documentation, and missing features. If you encounter any issues, :ref:`contact us <Support>`.

|GitHub release| |nbsp| |GitHub issues-open| |nbsp| |GitHub issues-closed| |nbsp| |GitHub pull-requests|

|DOI| |nbsp| |Documentation| |nbsp| |MIT license|

Known Issues
============

- Fails with GCC 8 (We suspect this is a GCC bug; see `issue #23 <https://github.com/jtebert/kilosim/issues/23>`_.)

  - **Workaround:** Change your GCC version.
- Viewer does not work over SSH

Citing
======

If you use this software, please cite it as follows:

**DOI (all/latest version):** `10.5281/zenodo.3406864 <https://doi.org/10.5281/zenodo.3406864>`_

**BibTeX:**

.. code-block:: bibtex

   bibtex
   @misc{kilosim_3406865,
   author       = {Ebert, Julia and
                     Barnes, Richard},
   title        = {Kilosim},
   month        = mar,
   year         = 2020,
   doi          = {10.5281/zenodo.3406865},
   url          = {https://doi.org/10.5281/zenodo.3406865}
   }

License
=======

This software is released under the `MIT License <https://github.com/jtebert/kilosim/blob/master/LICENSE)>`_.

Support
=======

If you are having issues installing or using the simulator or you have questions: `open an issue on GitHub <https://github.com/jtebert/kilosim/issues/new>`_ or `email Julia Ebert <mailto:julia\@juliaebert.com>`_.

.. |GitHub release| image:: https://img.shields.io/github/release-pre/jtebert/kilosim?color=yellow&style=for-the-badge
   :target: https://github.com/jtebert/kilosim/releases

.. |GitHub issues-closed| image:: https://img.shields.io/github/issues-closed/jtebert/kilosim.svg?style=for-the-badge
   :target: https://GitHub.com/Naereen/StrapDown.js/issues?q=is%3Aissue+is%3Aclosed

.. |GitHub issues-open| image:: https://img.shields.io/github/issues/jtebert/kilosim.svg?style=for-the-badge
   :target: https://github.com/jtebert/kilosim/issues

.. |GitHub pull-requests| image:: https://img.shields.io/github/issues-pr/jtebert/kilosim.svg?style=for-the-badge
   :target: https://GitHub.com/jtebert/kilosim/pull/


.. |Documentation| image:: https://readthedocs.org/projects/kilosim/badge/?version=latest&style=for-the-badge
   :target: https://kilosim.readthedocs.io/en/latest/?badge=latest

.. |DOI| image:: https://zenodo.org/badge/DOI/10.5281/zenodo.3406864.svg
   :target: https://doi.org/10.5281/zenodo.3406864

.. |MIT license| image:: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge
   :target: https://lbesson.mit-license.org/

.. |nbsp| unicode:: 0xA0