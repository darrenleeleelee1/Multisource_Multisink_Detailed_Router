#!/bin/bash
# build
make clean_out && make -j16
# declaring
start_index=0
end_index=99
# run
fault_files=""
reroute_files=""
log_path="tmp.log"
for i in $(seq $start_index $end_index); do
    echo "Running case test$i.txt"
    ./router ./case/test$i.txt ./out/test$i.txt >"$log_path" 2>&1
    if [ $? -ne 0 ]; then
        fault_files="$fault_files$i, "
    elif grep -q "reroute" "$log_path"; then
        reroute_files="$reroute_files$i, "
    fi
done
rm -f "$log_path"
if [ -n "$reroute_files" ]; then
  	echo "Reroute at test{$reroute_files}.txt"
fi
if [ -n "$fault_files" ]; then
  	echo "Fault at test{$fault_files}.txt"
fi
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
    not_run_files=""
    for i in $(seq $start_index $end_index); do
        echo "Running case test$i.txt."
        if [ ! -e ./out/test$i.txt ]; then
            not_run_files="$not_run_files$i, "
        elif (./verifier ./out/test$i.txt | grep -q "Error"); then
            echo "Error found in case test$i.txt."
            echo "Please run the following code to check the log."
            echo "./verifier ./out/test$i.txt"
            not_pass_files="$not_pass_files$i, "
        else
            echo "Passed."
        fi
    done
    if [ -n "$not_run_files" ]; then
        echo "Not run at test{$not_run_files}.txt"
    fi
    if [ -n "$not_pass_files" ]; then
        echo "Failed at test{$not_pass_files}.txt"
    fi
fi


# draw
if [ "$draw" = true ]; then
    python3 visual.py --dir ./out
fi