#!/bin/bash
# build
make clean && make -j16

# run
for i in {0..9}; do
    echo "Running case test$i.txt"
    ./router ./case/test$i.txt ./out/test$i.txt
done

# verify
if [ "$1" == "--verify" ]; then
    for i in {0..9}; do
        echo "Running case test$i.txt."
        ./verifier ./out/test$i.txt > ./out/log$i.txt
    done
    for i in {0..9}; do
        if grep -q "Error" ./out/log$i.txt; then
            echo "Error found in case test$i.txt."
            echo "Please run the following code to check the log."
            echo "./verifier ./out/test$i.txt"
        else
            echo "Passed."
        fi
    done
fi


# draw
if [ "$2" == "--draw" ]; then
    rm ./out/log**
    python3 visual.py --dir ./out
fi