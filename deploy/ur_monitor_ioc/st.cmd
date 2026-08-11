# UR robot monitoring IOC startup script -*- shell-script -*-
#
# Read-only dashboard + RTDE-receive slice of the epics-rs-iocs ur-robot
# IOC (urRobot port). Loads ONLY the two databases whose ports coexist
# with the robot-sequencer daemon:
#
#   - dashboard (TCP 29999): robot mode, safety status, program state
#   - RTDE receive (TCP 30004): joint/TCP state, safety word
#
# The control, io, jog and gripper databases are deliberately absent —
# their ports claim the program slot, RTDE input registers and the
# URCap gripper socket, all of which the sequencer (ur-driver +
# robotiq-hande) owns exclusively.
#
# Run with the ur-robot-ioc binary from the sibling epics-rs-iocs
# checkout; $(URROBOT) defaults to that crate's dir, so the db paths
# resolve regardless of cwd:
#
#   cargo run --release -p ur-robot-ioc -- st.cmd

epicsEnvSet("PREFIX", "Robot:UR:")
epicsEnvSet("IP", "192.168.192.10")

# Dashboard server, TCP 29999.
URDashboardConfig("dash", "$(IP)", 0.1)
dbLoadRecords("$(URROBOT)/db/dashboard.db", "P=$(PREFIX),PORT=dash")

# RTDE receive, TCP 30004. Read-only output subscription; URControl
# multiplexes RTDE across clients, so this coexists with the
# sequencer's own RTDE connection.
RTDEReceiveConfig("rtde_recv", "$(IP)", 0.02)
dbLoadRecords("$(URROBOT)/db/rtde_receive.db", "P=$(PREFIX),PORT=rtde_recv")

iocInit()
