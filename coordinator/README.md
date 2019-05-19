# The Coordinator Folder

This is the coordinator folder area. This folder hosts all the coordinator related objects:

    Coordinator Folder
    ├── coordinator                                     # Coordinator Code Package
    │   ├── CMakeLists.txt                              # CMake List
    │   ├── package.xml                                 # Package XML
    │   └── src                                         # Source Folder 
    │       ├── grab_tote_v2.cpp                        # LATEST grab FAKE_TOTE and REAL_TOTE
    │       └── visit_all.cpp                           # LATEST command the vehicle to go all station orderly
    └── README.md                                       # This File!

## Command Related to Coordinator

### Latest Command

1. **grab_tote:** `rosrun coordinator grab_tote_v2`
2. **visit_all:** `rosrun coordinator visit_all`

### Unused Command

### Unimplemented Command
