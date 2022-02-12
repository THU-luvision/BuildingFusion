#ifndef UTILS_H_
#define UTILS_H_

#include <torch/torch.h>

torch::Tensor toLongTensor(int dimension, int x);
#endif