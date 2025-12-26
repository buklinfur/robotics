# ex05_hare_bot — движение змейкой 

## Идея движения
Траектория «змейка» реализована как повторяемое чередование двух дуг с противоположной кривизной:

- **Лепесток 1:** постоянные `linear.x = v`, `angular.z = +w`
- **Лепесток 2:** постоянные `linear.x = v`, `angular.z = -w`


## Состав пакета
- `src/figure_eight.cpp` — узел, публикующий `/cmd_vel`
- `launch/figure_eight.launch.py` — запуск Gazebo + bridge + robot_state_publisher + узел движения

> В launch используются ресурсы робота из пакета **ex04_hare_bot** (URDF/world/bridge).

## Сборка
```bash
cd ~/workspace/ros2_ws
colcon build --packages-select ex05_hare_bot
source install/setup.bash
```

## Запуск
```bash
ros2 launch ex05_hare_bot figure_eight.launch.py
```

## Параметры узла
Задаются через параметры ROS2 (см. исходник `figure_eight.cpp`):

- `linear_x` (м/с), по умолчанию: `0.35`
- `angular_z` (рад/с), по умолчанию: `0.7` (используется модуль, знак переключается автоматически)
- `segment_sec` (сек), по умолчанию: `6.0`
- `rate_hz` (Гц), по умолчанию: `20.0`