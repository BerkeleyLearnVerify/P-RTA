"""from verifai.samplers.scenic_sampler import ScenicSampler
from dotmap import DotMap
from verifai.falsifier import generic_falsifier
import pickle


path_to_scenic_file = 'scene_description.sc'
sampler = ScenicSampler.fromScenario(path_to_scenic_file)

MAX_ITERS = 20
PORT = 8888
MAXREQS = 5
BUFSIZE = 8192

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.save_error_table = False
falsifier_params.save_good_samples = False

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

falsifier = generic_falsifier(sampler=sampler, sampler_type='scenic',
                              falsifier_params=falsifier_params,
                              server_options=server_options)

falsifier.run_falsifier()

print("Scenic Samples")
for i in falsifier.samples.keys():
    print("Sample: ", i)
    print(falsifier.samples[i])

"""
from verifai.samplers.scenic_sampler import ScenicSampler
from dotmap import DotMap
from verifai.falsifier import mtl_falsifier
from verifai.features.features import *
import pickle


#path_to_scenic_file = 'try.sc'
#sampler = ScenicSampler.fromScenario(path_to_scenic_file)

MAX_ITERS = 10
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.save_error_table = True
falsifier_params.save_good_samples = False

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

specification = ["G(battery)", "G(geofence1 & geofence2)"]

objects = Struct({'robot': Box([-0.2, 0.2], [-0.2, 0.2]),
                  'charger': Box([-0.2, 0.2], [-0.2, 0.2]),
                  'slave1': Box([0.3, 1.0], [-1.0, 1.0]),
                  'slave2': Box([0.3, 1.0], [-1.0, 1.0]),
                  'slave3': Box([-1.0, -0.3], [-1.0, 1.0]),
                  'slave4': Box([-1.0, -0.3], [-1.0, 1.0]),
                  'box1': Box([0.75, 1.0], [-0.75, 0.75]),
                  'box2': Box([0.75, 1.0], [-0.75, 0.75]),
                  'box3': Box([-1.0, -0.25], [-0.25, 0.25]),
                  'box4': Box([0.25, 1.0], [-0.25, 0.25]),
                  'box5': Box([-1.0, -0.75], [-0.75, 0.75]),
                  'box6': Box([-1.0, -0.75], [-0.75, 0.75])})

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




