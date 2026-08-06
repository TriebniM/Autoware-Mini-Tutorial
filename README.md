# Autoware Mini Tutorial

This tutorial is an introduction to [Autoware Mini](https://github.com/UT-ADL/autoware_mini). There are 8 lessons that walk you through the main modules of an autonomous driving stack.

## Lessons

| Lesson | Topic                   |
|--------|-------------------------|
|    1   | [Introduction to ROS](lesson1/README.md)     |
|    2   | [Localizer](lesson2/README.md)               |
|    3   | [Controller](lesson3/README.md)              |
|    4   | [Global planner](lesson4/README.md)          |
|    5   | [Obstacle detection](lesson5/README.md)      |
|    6   | [Local planner](lesson6/README.md)           |
|    7   | [Traffic light detection](lesson7/README.md) |
|    8   | [Testing in the CARLA simulator](lesson8/README.md) |

## Getting Started

### 1. Set up SSH keys for GitHub

You need an SSH key to push your work. If you don't have one yet:

1. [Generate a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent). **Set a passphrase** when prompted, so your key is protected on university computers.
2. [Add the public key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account). You can set the expiry to length of the course for extra security.

### 2. Fork and clone this repository

The university lab PCs already have the Autoware Mini workspace set up at `~/autoware_mini_ws/`. If you are working on your own machine, follow the setup instructions in the [Autoware Mini repository](https://github.com/UT-ADL/autoware_mini) first.

1. Go to this repository on GitHub and click **Fork** to create your own copy.
2. Clone your fork into the catkin workspace:
   ```bash
   cd ~/autoware_mini_ws/src
   git clone git@github.com:<your_github_username>/Autoware-Mini-Tutorial.git autoware_mini_tutorial
   ```
3. Follow the instructions in [Lesson 1](lesson1/README.md) to get started.

### Working with Git

We suggest regularly commiting and pushing your work. All assessment will be done by reviewing your code in your forked repository.
