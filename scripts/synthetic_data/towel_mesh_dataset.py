import argparse
import os
import subprocess
import sys

from tqdm import tqdm

dataset_size = 2

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1 :]  # get all args after "--"
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_size", type=int)
    args = parser.parse_known_args(argv)[0]
    dataset_size = args.dataset_size

script = "towel_mesh.py"

file_directory = os.path.dirname(os.path.realpath(__file__))
script_path = os.path.join(file_directory, script)

# We generate each sample is a separate process for robustness
print("Generating samples")
for seed in tqdm(range(dataset_size)):
    command = f"blender -b -P {script_path} -- --seed {seed}"
    subprocess.run([command], shell=True, stdout=subprocess.DEVNULL)
