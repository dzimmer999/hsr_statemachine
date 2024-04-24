# File explaination:
## States
### find_marker.py
Localizes the HSR using Markers, can be skipped if already aligned
### take_picture.py
Takes pictures in kitchen
### drive_to_destination.py
Drives to a destination passed by a string (=name of destiantion)
### calc_position.py
Calculates the position through the passed marker coordinates (can also be skipped if the HSR is already aligned)
### tf_broadcaster.py
Broadcastes the tf frames of the markers for alignment
### tf_calculator.py
Calculates the tf frames when new markers want to be used
### statemachine_*.py
Different statemachines depended on task