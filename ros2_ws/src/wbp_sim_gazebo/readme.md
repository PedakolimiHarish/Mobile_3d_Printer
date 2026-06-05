# wbp_sim_gazebo

Gazebo Harmonic world adapter for WBP simulation.

## Purpose

This package provides a **standalone simulation environment** for WBP.
It is intentionally decoupled from firmware, supervisor, and UI.

## Responsibilities

- Launch Gazebo Harmonic
- Load a deterministic world
- Spawn the WBP robot from `wbp_description`
- Publish simulation time

## Non-Responsibilities

- No controllers
- No localization
- No motion execution
- No firmware IPC

## Usage

```bash
ros2 launch wbp_sim_gazebo gazebo_world.launch.py
