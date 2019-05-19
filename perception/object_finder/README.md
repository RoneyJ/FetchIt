# The object_finder Package

This package is a "shell" for development of an object-finder action service.
In the goal message, specify a code corresponding to a known object type.
These codes are defined in part_code package located in the extern folder. More detailes are listed under dependencies

The response will contain a return code, e.g. for "object found", "object type not recognized", or "object not found".  

If object is found, there may be one or more.  Poses of objects will be in result message in a vector of object poses.

## Most Up to Date Node

The following node will be actively used, as of `2019-05-19`.

**object_finder_main** `rosrun object_finder object_finder_main`
This node is responsible for starting the object finder server. It is called from other nodes to "find an object".

## Included Nodes

Nodes here are included, but they might not be actively used for competition:

**example_object_finder_action_client** `rosrun object_finder example_object_finder_action_client` **(OBSOLETE)**
This node is an test node for specifically test for obejct finder. It is kept here but it can be replaced by interactive coordinator

## Structure

    Obejct Finder Folder
    ├── action                                        # Action server message structure
    │   └── objectFinder.action                       # Action Server Message Definition
    ├── include                                       # Library include structure
    │   └── object_finder_as                          # Action server include specifier
    │       └── object_finder.h                       # Header file for object finder
    ├── template                                      # tote angle calculator template folder structure
    │   └── new_template.jpg                          # tote angle calculator template
    ├── src                                           # source code folder
    │   ├── object_finder_main.cpp                    # Main file for starting the object finder action server
    │   *****MIX Compile*****                         # The following file are actually considered one file.
    │   ├── object_finder_as.cpp                      # Main file for Object Finder Action Server
    │   ├── object_finder_helper_fncs.cpp             # Helper function for object_finder_as
    │   ├── bolt_finder_fncs.cpp                      # Seperation file for finding bolt part. Combined towards Object_finder_as
    │   ├── gear_finder_fncs.cpp                      # Seperation file for finding gear part. Combined towards Object_finder_as
    │   ├── bolt_finder_fncs.cpp                      # Seperation file for finding bolt. Combined towards Object_finder_as
    │   ├── gearbox_finder_fncs.cpp                   # Seperation file for finding gearbox. Combined towards Object_finder_as
    │   ├── template_tote_finder.cpp                  # Seperation file for finding tote. Combined towards Object_finder_as
    │   *****END MIX COMPILE*****
    │   └── example_object_finder_action_client.cpp   # A test file for object finder
    ├── package.xml                                   # Package XML
    ├── CMakeList.txt                                 # CMake List
    └── README.md                                     # This File!

## Dependencies

This package generates library: `object_finder_as`

### Part Codes

Part Codes are follows:

    int32 FAKE_PART=1
    int32 GEARBOX_TOP=101
    int32 GEARBOX_BOTTOM=102
    int32 BOLT = 103
    int32 SMALL_GEAR =104
    int32 LARGE_GEAR = 105
    int32 TOTE = 106

TO utilize them, do: `part_codes::part_codes::GEARBOX_TOP`