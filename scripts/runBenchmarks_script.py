import os, re, subprocess, csv, argparse
from git import Repo
from statistics import mean

# To run this script properly, you must set up your benchmarking subcases using git
# Naming is important: 
# You must be on branch 'your_branch_name' - the script will not run your scene on this branch
# And must have named your benchmark branches as 'your_branch_name-test1', 'your_branch_name-test2' etc
# The script will check out these branches, run your scene and accumulate results

# Edit to your system
parser = argparse.ArgumentParser(description="Run benchmarks for a scene")
parser.add_argument("-sofaExe", type=str, default="runSofa", help="Path to runSofa executable in your system")
parser.add_argument("-scene", type=str, help="Path to scene file you wish to run")
parser.add_argument("-iterations", type=int, default=100, help="Number of ODE solver iterations to perform")
parser.add_argument("-tests", type=int, default=3, help="Number of tests to run")
args = parser.parse_args()

# get arguments
runSofa=args.sofaExe
# Scene name
xml_name = args.scene
# Runtime setup
n_iterations = args.iterations
# Number of tests to run for each case
n_tests = args.tests

# Dictionary to store results
benchmarks = {}
results = {'time': [] , 'fps' : [], 'iterations' : n_iterations, 'git-branch' : ''}

# Get git info to find branches
repo = Repo(search_parent_directories=True)
branch_prefix = repo.active_branch.name
benchmark_branches = [
    branch for branch in repo.branches if branch.name.startswith(branch_prefix + '-')]

print(f'Running {xml_name} spawned from {branch_prefix} with {n_iterations} iterations')

output_filename = 'log.performance.csv'
with open(output_filename, mode='w', newline='') as csv_file:
    csv_file.write(branch_prefix + ', time [s], fps\n')

    for branch in benchmark_branches:
        repo.git.checkout(branch.name)
        git_tag = branch.name[len(branch_prefix + '-'):]
        benchmarks[git_tag] = results
        benchmarks[git_tag]['git-branch'] = branch.name

        for i in range(n_tests):
            print(f'Git tag: {git_tag} - test {i+1}/{n_tests}')
        
            # This is the way to measure performance
            output = subprocess.run([runSofa, "-g", "batch", "-n", str(n_iterations), xml_name], shell=False, capture_output=True, text=True)
            for line in output.stdout.splitlines():
                if "iterations done in" in line:
                    numbers = re.findall(r"\d+\.\d+", line)
                    time_taken, fps = float(numbers[-2]), float(numbers[-1])
                    benchmarks[git_tag]['time'].append(time_taken)
                    benchmarks[git_tag]['fps'].append(fps)
                    break

            ## This is to troubleshoot in case SOFA crashes and no message is available
            #output = subprocess.Popen([runSofa, "-g", "batch", "-n", str(n_iterations), xml_name], shell=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            #for line in output.stdout:
            #    print(line, end="")
            #    if "iterations done in" in line:
            #        numbers = re.findall(r"\d+\.\d+", line)
            #        time_taken, fps = numbers[-2], numbers[-1]
            #        scenarios[scenario]['time'].append(time_taken)
            #        scenarios[scenario]['fps'].append(fps)
            #        break
            #output.wait()
    
        mean_time = mean(benchmarks[git_tag]['time'])
        mean_fps  = mean(benchmarks[git_tag]['fps'] )
        csv_file.write(f'{git_tag}, {mean_time}, {mean_fps}\n')
            
repo.git.checkout(branch_prefix)
