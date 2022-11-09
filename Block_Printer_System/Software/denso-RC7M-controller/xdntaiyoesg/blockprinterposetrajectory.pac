'!TITLE ""
PROGRAM BlockPrinterPoseTrajectory

#define SplineIndex 1

TakeArm

'limit forces if total force is not max value
if F0 + F1 + F2 + F3 + F4 + F5 < 600 then
  ST_SetFrcCoord 1
  ST_SetFrcLimit F0, F1, F2, F3, F4, F5
  ST_SetCompControl
end if

'set a spline path using global J variable
defint index
for index = 0 to I0 - 1
  SetSplinePoint SplineIndex, P[index]
next

move S, SplineIndex, speed = F6

ClrSplinePoint SplineIndex

if F0 + F1 + F2 + F3 + F4 + F5 < 600 then
  ST_ResetCompControl
  ST_ResetFrcLimit
end if

stop
stopend
END
