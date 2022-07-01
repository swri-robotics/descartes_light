^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package descartes_light
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added default state evaluator that treats all states equally (`#94 <https://github.com/swri-robotics/descartes_light/issues/94>`_)
  * Added default state evaluator that treats all states equally
  * Removed solution input parameter to pass checks
* Added CPack commands (`#93 <https://github.com/swri-robotics/descartes_light/issues/93>`_)
* Update CMake boilerplate tools version (`#92 <https://github.com/swri-robotics/descartes_light/issues/92>`_)
  * Updated to latest version of cmake boilerplate tools
  * Removed tesseract reference
  * Added option for building BGL enabled components
* Remove dependency on catkin (`#89 <https://github.com/swri-robotics/descartes_light/issues/89>`_)
* Contributors: Josh Langsfeld, Michael Ripperger, marrts


0.3.0 (2021-11-16)
------------------
* Add Solver interface and StateEvaluator interface by @Levi-Armstrong in https://github.com/swri-robotics/descartes_light/pull/50
* Reorganization + Fixes by @marip8 in https://github.com/swri-robotics/descartes_light/pull/51
* Euclidean distance edge scale by @marip8 in https://github.com/swri-robotics/descartes_light/pull/52
* Euclidean distance state evaluator by @marip8 in https://github.com/swri-robotics/descartes_light/pull/53
* Sampler Interface Update by @marip8 in https://github.com/swri-robotics/descartes_light/pull/54
* Various Updates by @marip8 in https://github.com/swri-robotics/descartes_light/pull/55
* Ladder Graph Error Checking by @marip8 in https://github.com/swri-robotics/descartes_light/pull/56
* Remove old kinematics utilities by @marip8 in https://github.com/swri-robotics/descartes_light/pull/58
* Clang Tidy Build for CI by @marip8 in https://github.com/swri-robotics/descartes_light/pull/59
* Add CMake Linter along with Contributing and License files. by @Levi-Armstrong in https://github.com/swri-robotics/descartes_light/pull/61
* Normalized cost evaluators by @marip8 in https://github.com/swri-robotics/descartes_light/pull/57
* Create State Base Class by @Levi-Armstrong in https://github.com/swri-robotics/descartes_light/pull/65
* Expose method for getting Eigen type in State class by @Levi-Armstrong in https://github.com/swri-robotics/descartes_light/pull/66
* Feature/boost graph by @colin-lewis-19 in https://github.com/swri-robotics/descartes_light/pull/67
* BGL Graph Serialization by @marip8 in https://github.com/swri-robotics/descartes_light/pull/68
* Parameterized Unit Test Type by @jrgnicho in https://github.com/swri-robotics/descartes_light/pull/70
* BGL Solver Revision by @marip8 in https://github.com/swri-robotics/descartes_light/pull/71
* BGL "efficient" Dijkstra Solver by @marip8 in https://github.com/swri-robotics/descartes_light/pull/72
* Event Visitors by @marip8 in https://github.com/swri-robotics/descartes_light/pull/73
* Change BGL solver predecessor map container by @colin-lewis-19 in https://github.com/swri-robotics/descartes_light/pull/75
* Dynamic Edge Adding Visitor by @colin-lewis-19 in https://github.com/swri-robotics/descartes_light/pull/76
* Visitor Templates for BGL Solver by @marip8 in https://github.com/swri-robotics/descartes_light/pull/80
* BGL Depth First Search by @colin-lewis-19 in https://github.com/swri-robotics/descartes_light/pull/77
* Revise organization by @marip8 in https://github.com/swri-robotics/descartes_light/pull/84
* Contributors: colin-lewis-19, jrgnicho, Levi Armstrong, Michael Ripperger

