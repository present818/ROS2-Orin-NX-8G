# ROS2 功能包导入指南（9.2.1 功能包导入）

---

## 步骤1：启动虚拟机并打开终端
启动虚拟机，点击系统桌面的终端图标，打开命令行终端。

## 步骤2：进入Home目录
点击桌面上的 `Home` 图标，进入用户主目录。

## 步骤3：导入功能包压缩包
找到本地“资源文件/3 功能包文件”中的 `simulations.zip` 压缩包，将其拖动到虚拟机的 `Home` 目录中。

## 步骤4：在Home目录中打开终端
在 `Home` 目录空白处右键，点击「Open in terminal」打开终端。

## 步骤5：创建ROS2工作空间目录
输入以下指令，创建ROS2工作空间及源码目录：
```bash
mkdir -p ~/ros2_ws/src
```

## 步骤6：解压并部署功能包
依次输入以下指令，解压压缩包并将功能包移动到工作空间的源码目录下：
```bash
# 解压压缩包
unzip ~/simulations.zip

# 将功能包移动到工作空间的 src 目录
mv ~/simulations ~/ros2_ws/src/simulations
```

## 步骤7：编译ROS2工作空间
输入以下指令编译功能包，等待编译完成：
```bash
cd ~/ros2_ws && colcon build --symlink-install
```

## 步骤8：移动配置文件 `.typerc`
输入以下指令，将 `.typerc` 配置文件移动到工作空间目录：
```bash
mv /home/ubuntu/.typerc ~/ros2_ws/.typerc
```

## 步骤9：验证文件移动是否成功
输入以下指令，查看工作空间目录下的文件：
```bash
cd ~/ros2_ws/ && ls -a
```
执行后，若输出列表中包含 `.typerc` 文件，则说明移动成功。

## 步骤10：配置自动加载环境变量
输入以下指令，将工作空间环境和配置文件添加到 `.bashrc`，实现终端启动时自动加载：
```bash
echo "source ~/ros2_ws/install/setup.bash">>~/.bashrc
echo "source ~/ros2_ws/.typerc">>~/.bashrc
```

## 步骤11：重新加载配置文件
输入以下指令，更新终端环境变量：
```bash
source ~/.bashrc
```

---
✅ 完成以上步骤后，功能包导入与环境配置全部完成，可正常使用。