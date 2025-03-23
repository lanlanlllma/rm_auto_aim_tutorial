# RM_AUTO_AIM 2024

## Debug

### Prepare

```shell
mkdir -p rm_auto_aim_ws/src && cd rm_auto_aim_ws
```

```shell
bash ./src/scripts/setup.bash
```

`.clang-format` 文件搭配 clang-format 插件进行代码格式化

`.clang-tidy` 文件使用 clangd 插件默认开启，使用 C/C++ 插件需要在 C_Cpp.codeAnalysis.clangTidy.enabled 中开启

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
