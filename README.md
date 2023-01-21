# 2023-robot

## TODO
Copy 2022 evergreen classes.  e.g. Joysticks, auto routine chooser, drivebase, etc.

Create Robot Constants:  
- Talon ID
- Pneumatic ID
- PDP ID

### Subsystems
Drivebase  
Shoulder  
Arm  
ClawRollers  
ClawPincher  
Targeting
Lights

### Commands
DriveToLevel - drive forward until the drive base angle is less than 10 degrees.  
DriveToCone2 - target the level 2 cone node and drive to it.  
DriveToCone3 - target the level 3 cone node and drive to it.  

ShoulderRotateTo(Degree): 0 is horizontal front, 90 is vertical up, 180/-180 horizontal back, -90 vertical down.  
ArmExtend(dist) - extend or retract the arm to a position.  
ClawRollersSet(+- power) - set the roller power to suck in or spit out.  
ClawPincherSet(open/closed)

SystemPlaceConeOn2 - rotate shoulder, extend arm, lower shoulder, open claw, retract arm.  
SystemPlaceConeOn3 - rotate shoulder, extend arm, lower shoulder, open claw, retract arm.  
SystemPlaceCubeOn2 - rotate shoulder, extend arm, lower shoulder, open claw, retract arm.  
SystemPlaceCubeOn3 - rotate shoulder, extend arm, lower shoulder, open claw, retract arm.  
SystemGetConeFromLoadingZone - with robot bumpers touching loading zone, back up, rotate shoulder, open claw, extend arm and claw rollers suck in, pinch, lift shoulder, retract arm, lower shoulder  



### AutoRoutines
`Auto_[Alliance]_[StartingLocation]_[StartingGamePiece]_[ScoringLocation]_[NextPiece]_[ScoringLocation]_[AutoLevel]`

#### Alliance
Red
Blue
Read this from the Driver Station

#### Starting Location
Grid 1 
Grid 2
Grid 3

#### Scoring Location
Cone L2/3 Pos 1/2/3/4/5/6
Cube L2/3 Pos 1/2/3

#### Next Piece
Cone 1,2,3,4
Cube 1,2,3,4

#### Scoring Location
Cone L2/3 Pos 1/2/3/4/5/6
Cube L2/3 Pos 1/2/3

#### Auto Level
Auto


AutoPlaceConeOn3AndAutoLevel
AutoPlaceConeOn2AndAutoLevel
AutoPlaceConeOn3
AutoPlaceConeOn2
AutoPlaceConeOn3Grab