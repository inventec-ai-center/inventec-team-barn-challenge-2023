#!/bin/bash
for i in $(eval echo {${1}..${2}}); do
    for j in {1..10} ; do            
        # run the test
        docker run --rm \
            --name="barn_${3}" \
            ros/melodic:barn-lflh \
            python3 run.py --world_idx $i --out_group ${3}
        echo $i ${3}
        sleep 2
    done
done
