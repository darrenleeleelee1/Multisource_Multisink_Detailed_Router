# Router
![workflow](https://github.com/darrenleeleelee1/Router/actions/workflows/result.yml/badge.svg?event=push)
![workflow](https://github.com/darrenleeleelee1/Router/actions/workflows/verifier.yml/badge.svg?event=push)
![workflow](https://github.com/darrenleeleelee1/Router/actions/workflows/memory_leak.yml/badge.svg?event=push)

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
