^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package descartes_light
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2025-01-14)
------------------
* Add ability to save graph as a dot graph (`#114 <https://github.com/swri-robotics/descartes_light/issues/114>`_)
* Contributors: Tyler Marr

0.4.3 (2024-06-01)
------------------
* Improve logging (`#110 <https://github.com/swri-robotics/descartes_light/issues/110>`_)
* Contributors: Tyler Marr

0.4.2 (2023-09-02)
------------------
* Fix debian source package
* Contributors: Levi Armstrong

0.4.1 (2023-09-02)
------------------
* Install headers with component name
* Contributors: Levi Armstrong

0.4.0 (2023-09-02)
------------------
* Update RICB and leverage components
* Ladder graph logging (`#105 <https://github.com/swri-robotics/descartes_light/issues/105>`_)
  * More detailed log of descartes ladder graph creation
  * Add format() and operator<< methods for waypoint sampler
  This makes it easier to get info on which sampler failed
  * Better failed vertex output
  * Output one sampled position when an edge fails
  Messy but code completion wouldn't work without being extra redundant in declarations.  Outputting 1 state is enough to pose the robot and see where it was trying to go.  It would be better to pull the original cartesian target pose out but that would be more difficult.
  * Changed printing to use ostream across Descartes logging
  * Fixed incorrect name of source file
  * Clang formatting
  * Fix new clang tidy warning
  * Fix bgl clang tidy warning
  * Fixed an accidental bug because I wasn't building with debug before
  * Updated printout to include a std::end so it actually prints
  * Revert unneeded clang tidy fix
  ---------
  Co-authored-by: Douglas Smith <douglas.smith@swri.org>
* Fixes  CXX version compilation issues of dependant projects (`#98 <https://github.com/swri-robotics/descartes_light/issues/98>`_)
  Co-authored-by: Roelof Oomen <roelof@LT-ROELOF.isogroup.local>
* Update/windows build (`#95 <https://github.com/swri-robotics/descartes_light/issues/95>`_)
  * Added test depends for gtest
  * Simplified clang-format job
  * Minor update to cmake format job
  * Updated windows build
  * Revised Ubuntu jobs
  * Use containers
  * Build in debug
  * Renamed CI files
  * Updated xenial CI name
  * Updated install of test library
  * Updated install location of benchmarks executable
  * Make install process non-interactive to avoid hang-ups in locale configuration
  * Remove testing from binary package jobs; add check to nuget build
  * Added colcon test result checks
  * Reintroduce Ninja
  * Simplify python installs
  * Remove second install of test library
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
  Co-authored-by: John Wason <wason@wasontech.com>
* Contributors: Levi Armstrong, Roelof, Tyler Marr

0.3.1 (2022-06-30)
------------------
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

