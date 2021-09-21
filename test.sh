colcon build
colcon test --pytest-args -v -m core
colcon test-result --verbose
