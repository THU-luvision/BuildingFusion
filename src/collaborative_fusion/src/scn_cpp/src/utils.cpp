#include "utils.h"

torch::Tensor toLongTensor(int dimension, int x)  {
    return torch::empty(dimension, torch::dtype(torch::kInt64)).fill_((long)x);
}
