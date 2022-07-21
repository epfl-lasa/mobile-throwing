# Throwing simulation

## Build the docker image
```bash
bash build-docker.sh
```

## Enter the docker container
```bash
bash run-docker.sh
```

## Inside the docker container, run the throwing simulation
```bash
python3 ms1_demo_mobile_manipulator_throw.py --box_x -3 --box_y 3 --box_z 0.5
```

Arguments are the location of the target box in the base frame of panda. Tested box height: [-0.5, 0.9]