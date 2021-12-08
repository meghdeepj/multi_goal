## Multigoal Planning in a Dynamic Environment


This project aimed to address planning in a dynamic environment with limited information about moving elements. The goal was to compile a system able to produce plans as close to optimal within the information available to the agent. A planning pipeline was developed to generate high-level task plans, low-level path plans, and to predict possible obstacle collisions. The resulting program proved capable of handling dynamic environments of complex geometry and complicated obstacle movement, while generating efficient plans. Such an use-case can be seen when planning for delivery robots or warehouse robots.

### Introduction

In many robotic environments, prior knowledge or initial surveyal can provide sufficient information about the static obstacles and terrain for optimal path planning. However, dynamic obstacles are often present and must be accounted for when generating optimal paths. Their paths and intents are almost never known to the robot, requiring analysis of available data to predict a trajectory. Changes in dynamic obstacle motion require reevaluation of the expected trajectory and potentially replan to avoid a collision.

Further, robots often must visit multiple locations to achieve their goal. Rather than planning paths independently and sequentially, which is likely to produce an overall suboptimal plan when unknown dynamic obstacles are present, plans can be generated from the robots current position to its remaining goal positions.

For this project, our team aimed to create a system to successfully plan paths in a 2D world with dynamic obstacles of unknown paths to a set of goal positions with no given order of completion. The system would be capable of achieving optimal paths within its knowledge and expectation of the environment at time of planning, be able to avoid collision with all obstacles, and do so in a live simulation, where taking too long to plan can lead to obstacle collision.
