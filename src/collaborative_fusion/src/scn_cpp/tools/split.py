        # save weights

import torch
import numpy as np




i = 0
print("Saving weights to npy files...")
for param_tensor in net.state_dict():
	print(param_tensor, "\t", net.state_dict()[param_tensor].size())
	np.save('weight/unet_' + str(i) + '.npy', net.state_dict()[param_tensor].cpu().detach().numpy())
	i = i + 1
raise ValueError("Model Loaded")

