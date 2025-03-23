# RM_AUTO_AIM 2024

## Debug

### Prepare

```shell
mkdir -p rm_auto_aim_ws/src && cd rm_auto_aim_ws
```

```shell
bash ./src/scripts/setup.bash
```

### Compile

```shell
colcon build --symlink-install
source ./install/setup.bash
```

### Run

```shell
bash ./scripts/run.bash
bash ./scripts/debug.bash
```
