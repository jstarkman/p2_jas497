# p2_jas497

Response to problem set number two.

## Example usage

With a `stdr_*` GUI launched, run both of these nodes (*N.B.* bring
them online in this order.  RC depends on the warnings from LA):

```bash
$ rosrun p2_jas497 p2_lidar_alarm
$ rosrun p2_jas497 p2_reactive_commander
```

## More documentation

See `report.(tex|pdf)` for the brief write-up.

# Prompt (copied verbatim and refomatted for Markdown)

## PS2

For this assignment, you should upgrade the "lidar_alarm.cpp" code in
the provided "lidar_alarm" package to make it smarter.

Instead of relying on only a single "ping" to detect obstacles in the
robot's path, it should examine an entire "corridor".

Run your revised lidar_alarm node together with the STDR simulator and
a reactive motion commander.  At a minimum you can use the
"reactive_commander.cpp" in the provided "stdr_control" package.
Better still, you could (optionally) make this commander more
intelligent (or at least more interesting).

## Deliverables

- Your code, in package form, with CMakeLists.txt, package.xml, and
  source in a *ROS package* form (*Do NOT just submit the CPP file!*)

- A brief report describing your theory of operation (your algorithm's
  logic for lidar_alarm, and, optionally, reactive_commander)

- a Kazaam [sic] movie (*.mp4) of your nodes running the STDR
  simulator.

*Please ZIP all of these things up in a folder titled "p2_caseID.zip"
(ex. p2_abc123.zip)*
