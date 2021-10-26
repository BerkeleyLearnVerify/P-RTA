from verifai.samplers.scenic_sampler import ScenicSampler
from dotmap import DotMap
from verifai.falsifier import mtl_falsifier
from verifai.features.features import *
import pickle


#path_to_scenic_file = 'try.sc'
#sampler = ScenicSampler.fromScenario(path_to_scenic_file)

MAX_ITERS = 100
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.save_error_table = True
falsifier_params.save_good_samples = False

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

specification = ["G(battery)"]

objects = Struct({'robot': Box([-0.5, 0.5], [-0.5, 0.5]), 'charger': Box([-0.25, 0.25], [-0.25, 0.25])})

sample_space = {'objects':objects}

falsifier = mtl_falsifier(sample_space=sample_space, sampler_type='ce',
                          specification=specification,
                          falsifier_params=falsifier_params,
                          server_options=server_options)

falsifier.run_falsifier()

print("Scenic Samples")
for i in falsifier.samples.keys():
    print("Sample: ", i)
    print(falsifier.samples[i])


print("Unsafe samples: Error table")
print(falsifier.error_table.table)


# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))




