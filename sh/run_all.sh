# build
make clean && make -j16
# run
for i in {0..9}; do
    echo "Running case test$i.txt"
    ./router ./case/test$i.txt ./out/test$i.txt
done
# draw
if [ $1 == "--draw" ]
then
    python3 visual.py --dir ./out
fi