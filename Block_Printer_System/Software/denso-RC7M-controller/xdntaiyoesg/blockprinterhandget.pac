'!TITLE "title"
PROGRAM BlockPrinterHandGet

'dummy variable(never used)
DEFINT state(11)

'10 ms
DO
  'set current hand position to F9 global variable
  CALL HAND_GETSTATE(0,state(),F9)
  DELAY 10
LOOP

STOP
STOPEND

END
