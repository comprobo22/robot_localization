# Description of Bag Files
This directory consists of 5 bag files that were used/collected in order to test and showcase the particle filter developed during this project. This is a short description of the functionality of each file.

## macfirst_floor_take_1 and macfirst_floor_take_2
These two bag files were provided by our instructor, Paul Ruvolo. They were recorded on the TurtleBot4 and were used while debugging the particle filter when testing the particle filter on the first floor of the MAC map.

## pf_gauntlet
This bag file is the successful localization of the Neato within the final gauntlet world when given no starting position. 

## pf_mac_first_floor
This bag file is the successful localization of the TurtleBot4 within a map containing one half of the first floor of Olin's Miller Academic Center when given no initial position.

## pf_mac_full_close
This bag file is the almost successful localization of the TurtleBot4 within a map containing all four floors of Olin's Miller Academic Center (each floor was placed side-by-side in the map) given no initial position. By the end of the trial, the particle filter goes back and forth between two different potential positions, one of which is the correct position, but fails to consistently converge at that point. This result suggests that either a different likelihood function is necessary that is better adept to the subtle nuances of the MAC's space, or more data (longer bag files) is needed for the particle filter to converge on the actual position of the robot.