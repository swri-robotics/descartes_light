/*
 * benchmark_utils.hpp
 *
 *  Created on: Jul 28, 2021
 *      Author: Jorge Nicho
 *
 */

/*
Copyright 20xx Southwest Research Institute

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef TEST_DESCARTES_LIGHT_BENCHMARKS_BENCHMARK_UTILS_HPP_
#define TEST_DESCARTES_LIGHT_BENCHMARKS_BENCHMARK_UTILS_HPP_

#include <string>
#include <chrono>
#include <iostream>

namespace descartes_light
{
namespace benchmark
{
template <class ReturnArg>
class PerformanceResults
{
public:
  PerformanceResults() {}

  ~PerformanceResults() {}

  void print(const std::string& header)
  {
    std::cout << header << std::endl;
    std::cout << "\tTime elapsed: " << std::setprecision(5) << time_elapsed << " secs" << std::endl;
    std::cout << "\tMemory Used: " << std::setprecision(5) << memory_used << std::endl;
  }

  double time_elapsed = 0.0;
  double memory_used = 0.0;

  ReturnArg return_val;
};

template <class ReturnArg>
static PerformanceResults<ReturnArg> measurePerformance(std::function<ReturnArg()> funct)
{
  using namespace std::chrono;
  PerformanceResults<ReturnArg> performance_results;

  auto start_time = steady_clock::now();
  performance_results.return_val = funct();
  auto end_time = steady_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;

  // saving performance results
  performance_results.time_elapsed = diff.count();
  return performance_results;
}

}  // namespace benchmark
}  // namespace descartes_light

#endif /* TEST_DESCARTES_LIGHT_BENCHMARKS_BENCHMARK_UTILS_HPP_ */
