#!/bin/bash
# build
make clean && make -j16
# declaring
start_index=0
end_index=99
# run
for i in $(seq $start_index $end_index); do
    echo "Running case test$i.txt"
    ./router ./case/test$i.txt ./out/test$i.txt
done

# parse aruguments
while getopts ":dv" opt; do
  case $opt in
    d)
      draw=true
      ;;
    v)
      verify=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

# verify
if [ "$verify" = true ]; then
    not_pass_files=""
    for i in $(seq $start_index $end_index); do
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
if [ "$draw" = true ]; then
    python3 visual.py --dir ./out
fi