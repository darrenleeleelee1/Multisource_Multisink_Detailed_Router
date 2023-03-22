# Router
![workflow](https://github.com/darrenleeleelee1/Router/actions/workflows/result.yml/badge.svg?event=push)
![workflow](https://github.com/darrenleeleelee1/Router/actions/workflows/verifier.yml/badge.svg?event=push)
![workflow](https://github.com/darrenleeleelee1/Router/actions/workflows/memory_leak.yml/badge.svg?event=push)

## Build and Execute
* Build   : `make` or `./sh/test.sh -b`
    * need `./obj` directory
* Execute : `./router <testcase> <output>` or `./sh/test.sh -r`
* Draw    : `./visual.py -i <result directory> -o <draw directory>` or `./sh/test.sh -d`
* Verify  : `./verifier <output>` or `./sh/test.sh -v`


## Flow
1. Two-pin nets based on the topology of RMST
2. Multisource multisink mazerouting
3. Rip-up and reroute stuck pins
