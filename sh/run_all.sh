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
    not_pass_files=""
    for i in {0..9}; do
        echo "Running case test$i.txt."
        if (./verifier ./out/test$i.txt | grep -q "Error"); then
            echo "Error found in case test$i.txt."
            echo "Please run the following code to check the log."
            echo "./verifier ./out/test$i.txt"
            not_pass_files="$not_pass_files$i, "
        else
            echo "Passed."
        fi
    done
    if [ -n "$not_pass_files" ]; then
        echo "Failed at test{$not_pass_files}.txt"
    fi
fi


# draw
if [ "$2" == "--draw" ]; then
    rm ./out/log**
    python3 visual.py --dir ./out
fi