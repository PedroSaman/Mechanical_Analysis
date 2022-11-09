'!TITLE "title"
PROGRAM BlockPrinterHandSet

'move hand to the position specified in F10 global variable (with max speed)
CALL LIB.X_CMMVABSPOS(0,F10,100,-1,0)

STOP
STOPEND

END
