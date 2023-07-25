# INVENTEC BARN Challenge 2023 - Simulation

<p align="center">
  <img width = "100%" src='nav-competition-icra2023/res/BARN_Challenge.png' />
</p>

## Requirements
* Ubuntu 20.04
* Docker
* Singularity | **(For submission purposes)**


---
## Installation
### Docker

Follow the instruction below to run simulations in Docker containers.
1. Follow this to install Docker.
```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker ${USER}
reboot
```

2. Build the docker image
   
```
cd lflh/docker/
sh create_lflh_image.sh
```

### Singularity
Follow the instruction below to run simulations in Singularity containers.

1. Follow this [instruction](Singularity-Install.md) to install Singularity

2. Build Singularity image (sudo access required)

Before building singularity image, remember to change the shebang of following python files under `lflh/scripts/` `lflh_utils.py`, `run_policy.py`, & `utils.py` with `#!/venv/bin/python3`
```
cd nav-competition-icra2023/
sudo singularity build --notest nav_competition_image.sif Singularityfile.def
```
---

## Run Simulations
### Development
If you run with GUI for development
```
cd lflh/docker/
xhost +
sh create_lflh_container.sh
docker attach barn_dev
python3 run.py --world_idx 0 --gui
```
* to detach container without stopping `CTRL+p, CTRL+q`
* to detach container with stopping `CTRL+d`

### Parallelism 
If you want run parallelism, you can specify the number of parallel process `n_parallel` in the `run_parallel_container.sh` to benchmark the codebase over all the environment test case with using all the cpu resources. *Especially running in the big cpu server like arceus or lugia.*
```
cd lflh/docker/
chmod +x run_lflh_parallel_container.sh
./run_lflh_parallel_container.sh XX
```

BARN challenge comprises only two subset environments to test: the 50 or 300 environemnts.

```
./run_lflh_parallel_container.sh 50 # for 50 environments
./run_lflh_parallel_container.sh 300 # for 300 environments
```

### Singularity benchmark
If you run it in a Singularity container:
```
cd nav-competition-icra2023/
./singularity_run.sh nav_competition_image.sif python3 run.py --world_idx 0
```
or run all over the environments test
```
./singularity_run.sh nav_competition_image.sif ./test_50.sh
or
./singularity_run.sh nav_competition_image.sif ./test_300.sh
```

## Report the result
Once the tests finished, run following script to report the test.

if you run single test benchmark:
```
cd nav-competition-icra2023/
python3 report_test.py --out_path out.txt
python3 plot_data.py --out_path out.txt
```

if you run parallel test benchmark, `summarize.py` will combine all '`out_*.txt`' and run the plot and report scripts simultaneously.
```
cd nav-competition-icra2023/
python3 summarize.py
```

You should see the report as this:
> Avg Time: 14.0344, Avg Metric: 0.2445, Avg Success: 0.9900, Avg Collision: 0.0060, Avg Timeout: 0.0040

**Please note** that the following results were obtained from our submitted solution, which was rigorously tested on a set of 50 environments provided by the competition organizers. 


## Copyright
CITE our technical report.
```
@article{mandala2023barn,
  title={The BARN Challenge 2023--Autonomous Navigation in Highly Constrained Spaces--Inventec Team},
  author={Mandala, Hanjaya and Christmann, Guilherme},
  journal={arXiv preprint arXiv:2307.14580},
  year={2023}
}
```

Also, reference to original author.
```
@inproceedings{wang2021agile,
  title={From agile ground to aerial navigation: Learning from learned hallucination},
  author={Wang, Zizhao and Xiao, Xuesu and Nettekoven, Alexander J and Umasankar, Kadhiravan and Singh, Anika and Bommakanti, Sriram and Topcu, Ufuk and Stone, Peter},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={148--153},
  year={2021},
  organization={IEEE}
}
```