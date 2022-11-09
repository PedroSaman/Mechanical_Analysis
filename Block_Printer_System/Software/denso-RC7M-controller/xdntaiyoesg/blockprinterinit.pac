'!TITLE "title"
PROGRAM BlockPrinterInit

'initialize hand state
CALL Hand_Org(0,0)
CALL lib.x_cmOnOffMotor(0,1)

STOP
STOPEND
END
