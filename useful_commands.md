# Useful Commands

### Build
```bash
# build
colcon build --symlink-install

# update context (so you can interact with the system)
# # cd src
source install/setup.bash
```

### rviz2
```bash
# you can run this in a separate terminal, but it is better to integrate with launch file
ros2 run rviz2 rviz2
```

### rqt_console
```bash
ros2 run rqt_console rqt_console
```

---

## Troubleshooting

### When rviz2 freezes up
"This is a common issue in rviz2. When you close Rviz, it saves the current window layout and geometry (including the fullscreen state) to a configuration file. If that configuration becomes corrupted or if the fullscreen mode conflicts with your window manager (common with Nvidia drivers or Wayland), the application can freeze upon relaunch."
```bash
# Backup and remove the default configuration file
mv ~/.rviz2/default.rviz ~/.rviz2/default.rviz.bak
```