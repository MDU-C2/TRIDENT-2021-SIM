# Changelog

## [20.10.6] - 14-04-2021

* `cola2_control`: solve issue in pilot section min surge velocity

## [20.10.5] - 07-04-2021

* `cola2_control`: improve teleoperation behavior when ack is lost

## [20.10.4] - 22-03-2021

* `cola2_safety`: solve safety issue with ros::Time zero
* `cola2_control`: solve issue with ros::Time zero in pilot and captain nodes
* `cola2_nav`: solve issue with ros::Time zero in navigator node
* `cola2_control`: solve compiler warnings in Ubuntu 20.04

## [20.10.3] - 04-03-2021

* `cola2_safety`: increased GPS surface timeout in navigator safety rule
* `cola2_safety`: solved issues in unit tests

## [20.10.2] - 11-01-2021

* `cola2_control`: solve Python compatibility issues with Ubuntu 20.04.
* `cola2_log`: solve Python compatibility issues with Ubuntu 20.04.

## [20.10.1] - 07-01-2021

* `cola2_nav`: forget usbl position history when reloading parameters or changing NED origin.

## [20.10.0] - 26-10-2020

* `cola2_log`: Added shutdown logger
* `cola2_control`: Pilot and captain simplification. Now using a single actionlib. The goto and section services have also been updated.
* `cola2_nav`: Keep full pose history instead of only positions `xy`.
* `cola2_nav`: Ease inheritance to implement custom navigators on top of the one available.
* Improved general diagnostics using the new diagnostic helper class
* `cola2_safety`: New package using C++, and without vehicle status.
* `cola2_control`: Solved issue with disable mission with no mission name when not in mission state.
* `cola2_control`: Added first version of open loop node.
* `cola2_sim`: The dynamics node now uses the new setpoints selector from cola2_lib_ros.
* `cola2_control`: New set zero velocity in the controller.

## [3.2.5] - 28-08-2020

* `cola2_nav`: Solved negative pressure check issue in pressure sensor callback.

## [3.2.4] - 18-03-2020

* `cola2_control`: Corrected issue in section services.

## [3.2.3] - 30-01-2020

* `cola2_nav`: Corrected magnetic declination sign so the magnetic declination is properly taken into account into the navigation filter. Magnetic declinations from websites such as [http://www.magnetic-declination.com/](http://www.magnetic-declination.com/) can be now used with the sign given there.

## [3.2.2] - 29-01-2020

* `cola2_safety`: Solved issue in disable mission service of recovery actions node.

## [3.2.1] - 10-01-2020

* `cola2_nav`: Added missing dependency to be able to compile without installing cola2_lib in the system (related to cola2_lib hotfix 3.2.2).

## [3.2.0] - 22-10-2019

* Add `_node` to all nodes source code files
* Add tags to `README.md` for auto-documentation
* Apply `cola2_lib` refactor changes
* Change `std_srvs/Empty` services to `std_srvs/Trigger`
* Delete node name from example configurations
* Keep nodes alive for auto-documentation
* `cola2_comms`: new acoustic command to reset vehicle timeout
* `cola2_control`: added mission pause and resume services
* `cola2_control`: added state_feedback topic to publish information about on-going state for all services
* `cola2_control`: all captain services are nonblocking now
* `cola2_control`: captain service calls now have a timeout of 5 seconds to ensure captain is not blocked
* `cola2_control`: captain_node is now single-threaded
* `cola2_control`: changed captain_status topic to reflect the internal state of the captain and the list of loaded missions
* `cola2_control`: enable/disable mission services can enable/disable a mission by name
* `cola2_control`: improve checks on mission validity before starting the mission
* `cola2_control`: keyboard_node is now single-threaded
* `cola2_control`: removed default mission nonblocking service from captain
* `cola2_log`: adapt default_param_handler to take filename as node name
* `cola2_log`: default_param_handler service to update single parameter
* `cola2_nav`: add service to only reload NED info
* `cola2_nav`: delete navigator/altitude_filtered topic (altitude output of navigator now only in navigator/navigation)
* `cola2_nav`: solve bug in too old USBL updates
* `cola2_sim`: dynamics, solve bug in negative thruster setpoints

## [3.1.1] - 15-04-2019

* Solved bug when loading mission actions with parameters

## [3.1.0] - 25-02-2019

* First release
