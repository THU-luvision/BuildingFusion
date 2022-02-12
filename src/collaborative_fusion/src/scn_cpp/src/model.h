#ifndef MODEL_H_
#define MODEL_H_

#include <torch/torch.h>
#include "sparseconvnet/SCN/sparseconvnet.h"
#include <string>
#include <vector>
#include <map>
#include <tuple>

using Tensors = std::vector<torch::Tensor>;
extern int weight_count;
/**
 * SparseConvNetTensor
*/
class SparseConvNetTensor{
public:
    torch::Tensor features;     /*FloatTensor*/
    std::shared_ptr<Metadata<3>> metadata;
    torch::Tensor spatial_size; /*LongTensor*/

    SparseConvNetTensor()   {
    }

    SparseConvNetTensor(torch::Tensor features_, 
                        std::shared_ptr<Metadata<3>> metadata_, 
                        torch::Tensor spatial_size_)   {
        features = features_;
        metadata = metadata_;
        spatial_size = spatial_size_;
    }

    torch::Tensor cuda()  {
        return features.cuda();
    }

    torch::Tensor cpu()  {
        return features.cpu();
    }
};


class Sequential : public torch::nn::Module
{
public:
    Sequential();
    template<typename ModuleType>
    Sequential& add(ModuleType module);

    Sequential& add();

    SparseConvNetTensor forward(SparseConvNetTensor input);

    torch::nn::Sequential _modules;
};

struct UNet : torch::nn::Module {
public:
    UNet(std::string weights_path);
    torch::Tensor forward(torch::Tensor coords, torch::Tensor features);
    
private:
	torch::Device *_device;
	// Sequential seq;
	torch::nn::Sequential seq;
};

// Instance
struct LearningBWDenseUNet : torch::nn::Module {
public:
    LearningBWDenseUNet(std::string weights_path);
    Tensors forward(torch::Tensor coords, torch::Tensor features);
    
private:
	torch::Device *_device;
	// Sequential seq;
    // UNet extractor
    torch::nn::Sequential instance_dense;
    // linear layers
	torch::nn::Linear linear = nullptr;
    torch::nn::Linear fc_regress = nullptr;
    torch::nn::Linear linear_regress = nullptr;
    torch::nn::Linear fc_embedding = nullptr;
    torch::nn::Linear linear_embedding = nullptr;
    torch::nn::Linear fc_displacement = nullptr;
    torch::nn::Linear linear_displacement = nullptr;
    //bw & occupancy
    torch::nn::Linear fc_bw = nullptr;
    torch::nn::Linear linear_bw = nullptr;

    torch::nn::Linear fc_occupancy = nullptr;
    torch::nn::Linear linear_occupancy = nullptr;
};

#endif
