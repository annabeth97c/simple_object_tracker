#include "simple_object_tracker/hungarian_algorithm.hpp"
#include <stdio.h>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  // Generate random sizes for the outer and inner vectors.
  const size_t num_rows = std::rand() % 100 + 1;
  const size_t num_cols = std::rand() % 100 + 1;

  // Create the cost matrix.
  std::vector<std::vector<double>> cost(num_rows, std::vector<double>(num_cols));

  // Fill the cost matrix with random values.
  std::generate(cost.begin(), cost.end(), [num_cols]() {
    std::vector<double> row(num_cols);
    std::generate(row.begin(), row.end(), []() { return (std::rand() / double(RAND_MAX)); });
    return row;
  });

  std::cout << "INPUT : " << std::endl;

  for (int r = 0; r < num_rows; r++)
  {
  	for (int c = 0; c < num_cols; c++)
  	{
  		std::cout << cost[r][c] << " ";
  	}
  	std::cout << std::endl;
  }

  // Call the function you want to fuzz with the cost matrix.
  std::vector<int> result = hungarian(cost);

  std::cout << "output : ";

  for (int r : result) {
    // Print the result values, or perform other actions
    std::cout << r <<" ";
  }

  std::cout << std::endl;

  return 0;
}