
#ifndef _ARRAYIO_H_
#define _ARRAYIO_H_

#include <Eigen/Core>
#include "config.h"

void saveArray(const ArrayXXb &arr, const std::string &fname);
void loadArray(ArrayXXb &arr, const std::string &fname);
bool arrayExists(const std::string &fname);

#endif
