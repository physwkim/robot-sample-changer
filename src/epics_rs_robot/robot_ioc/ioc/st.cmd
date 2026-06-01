# Robot soft-record IOC startup script.
# Replaces: softIoc -d db/robot.db
#
# ROBOT_DB defaults to /home/bl9b/ws/db (set in main.rs); override via env.

dbLoadRecords("$(ROBOT_DB)/robot.db")

# Autosave: persist robot run-state across IOC/power restart (resume-after-crash).
# Values are restored at iocInit (pass 1) and saved on change, every 1 s.
set_savefile_path("$(ROBOT_IOC)/autosave")
set_requestfile_path("$(ROBOT_IOC)/autosave")
set_pass1_restoreFile("robot_state.req")
create_monitor_set("robot_state.req", 1)

iocInit()

# Handy at the iocsh prompt:
#   dbl
#   camonitor Robot:Trigger Robot:CurrentStep
