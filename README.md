# EJUST ROV Simulation Mission
## Mission 1 Video
 [![Watch the video](https://github.com/a-alnaggar/ROV_EJUST/assets/121443735/9813af0a-c863-448c-a716-6eeef19e82e1](https://github.com/a-alnaggar/ROV_EJUST/blob/2bd3ce2f462b03ab4b0e5ab1b2ab7921fb011ce6/WhatsApp%20Image%202023-10-09%20at%206.47.50%20PM.jpeg)](https://www.youtube.com/watch?v=QihYc0L7ufg)
## Autonomous Mission Video
[![Watch the video](https://github.com/a-alnaggar/ROV_EJUST/blob/880b62635f2593fc961e05d001b4f6ad1f2e061f/WhatsApp%20Image%202024-01-03%20at%205.58.46%20PM.jpeg)](https://youtu.be/KmTFxljsGuw)

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
