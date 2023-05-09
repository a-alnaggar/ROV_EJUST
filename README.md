# General Instructions

To contribute:
1. Checkout to a new branch
2. Push your code
3. Make pull request and add needed reviewers

# Adding Packages

All packages should reside in `ROV-nodes` and named `<node_name>-node`.

Add a the package name and description in `ROV-nodes/README.md` following the same style.

Add a `README` file if the node requires external packages.

# Dockerized ROS
_If you already have `ROS` installed ignore this section_

In `docker-compose.yaml` adjust volume bindings:

```yaml
    volumes:
      - <PATH TO YOUR CATKIN WORKSPACE>:/opt/catkin_ws
      - ./ros_env_init.sh:/opt/ros_env_init.sh
```
