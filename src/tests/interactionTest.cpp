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

void idEncodingTest() {
  std::cout << "IdEncoding test\n";
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
  std::cout << "IdEncoding test" << std::endl;
}

bool comparePoseCosts(const PoseCost &p, const PoseCost &q) {
  bool success1 = true;
  {
    if (p.id != q.id || p.jointCost != q.jointCost ||
        p.interactionCount != q.interactionCount) {
      success1 = false;
      std::cout << "Initial matches fail\n";
    }
    for (Eigen::Index i = 0; i < p.jointInteractionCostVec.size(); ++i) {
      if (p.jointInteractionCostVec(i) != q.jointInteractionCostVec(i)) {
        std::cout << i << " " << p.jointInteractionCostVec(i) << " "
                  << q.jointInteractionCostVec(i) << "cost vec matches fail\n";
        success1 = false;
        break;
      }
    }
  }
  return success1;
}
void poseCostBinaryStreamTest() {
  std::cout << "PoseCost Streaming test" << std::endl;
  PoseCost p(3.1455, 1132364635);

  p.jointCost = 0.1214324;
  for (Eigen::Index i = 0; i < p.jointInteractionCostVec.size(); ++i) {
    p.jointInteractionCostVec(i) = i;
  }
  p.updateInteractionCount();

  PoseCost r(10.4367, 178721316710679743);

  r.jointCost = 0.1214324;
  for (Eigen::Index i = 0; i < p.jointInteractionCostVec.size(); ++i) {
    r.jointInteractionCostVec(i) = i;
  }
  r.updateInteractionCount();

  std::stringstream input;
  p.write(input);
  r.write(input);

  std::stringstream out1;
  PoseCost q(0, 0);
  PoseCost s(0, 0);
  q.read(input);
  s.read(input);
  q.write(out1);
  s.write(out1);

  if (comparePoseCosts(p, q) && comparePoseCosts(r, s) &&
      input.str().compare(out1.str()) == 0) {
    std::cout << "Test success" << std::endl;
  }
  std::cout << "PoseCost Streaming test END" << std::endl;
}

int main() {
  idEncodingTest();
  poseCostBinaryStreamTest();

}
