#include <algorithm>
#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <thread>

#include "Interactions.hpp"
#include "utils.hpp"

using namespace Interactions;

int main() {

  for (size_t i = 1; i < 20; ++i) {
    std::cout << i << "\t";
    for (size_t j = 0; j < i; ++j) {
      std::cout << "* ";

      size_t idx = encodeInteractionVecIndex(i, j);
      size_t row = 0, col = 0;
      decodeInteractionVecIndex(idx, row, col);
      if (idx != encodeInteractionVecIndex(row, col)) {
        std::cout << "SOMETHING IS WRONG!!!" << std::endl;
      }
    }
    std::cout << "\n";
  }
  std::cout << std::endl;
}
