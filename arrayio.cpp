
#include <fstream>
#include "arrayio.h"

void saveArray(const ArrayXXb &arr, const std::string &fname) {
  std::ofstream file(fname);
  if (file.is_open())
  {
    for (int i = 0; i < arr.rows(); i++) {
      for (int j = 0; j < arr.cols(); j++) {
        file << arr(i,j);
      }
    }
  }
  file.close();
}

void loadArray(ArrayXXb &arr, const std::string &fname) {
  std::ifstream file(fname);
  if (file.is_open())
  {
    for (int i = 0; i < arr.rows(); i++) {
      for (int j = 0; j < arr.cols(); j++) {
        arr(i,j) = file.get();
      }
    }
  }
  file.close();
}

bool arrayExists(const std::string &fname) {
  std::ifstream file(fname);
  bool res = file.is_open();
  file.close();
  return res;
}
