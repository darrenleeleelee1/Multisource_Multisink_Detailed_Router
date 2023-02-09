# Router
## Build and Execute

* Build   : `make`
    * need `./obj` directory
* Execute : `./router <testcase> <output>`
* Draw    : ` python3 visual.py --dir <output>`
* Run all and draw : `./sh/run_all.sh [--draw]`
    * need `./out` directory  


## Flow
1. Two-pin nets based on the topology of RMST
2. Multisource multisink mazerouting
3. Rip-up and reroute stuck pins
