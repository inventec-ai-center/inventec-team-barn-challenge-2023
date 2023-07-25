#!/bin/bash
n_parallel=2
total=$1
n_proc=$((total / n_parallel))
for i in $(eval echo {1..$n_parallel}); do
    ll=$(( (i-1) * n_proc))
    ul=$((i * n_proc - 1))
    ./lflh_${total}_world.sh $ll $ul $i &
    pids[${k}]=$!
    # echo $i $ll $ul
done

# wait for all pids
for pid in ${pids[*]}; do
    wait $pid
done

remainder=$((total - ul))
if [[ $remainder -gt 1 ]]
then
    ll=$((ul+1))
    ul=$((total - 1))
    i=$((i + 1))
    ./lflh_${total}_world.sh $ll $ul $i &
    # echo $i $ll $ul
fi
