#!/bin/bash
for i in $(eval echo {${1}..${2}}); do
    n=$((i * 6)) # 50 test BARN worlds with equal spacing indices: [0, 6, 12, ..., 294]
    for j in {1..10} ; do            
        # run the test
        docker run --rm \
            --name="barn_${3}" \
            --volume="${HOME}/Workspaces/inventec_ws/src/nav-competition-icra2023:/root/jackal_ws/src/nav-competition-icra2023/:rw" \
            ros/melodic:barn-lflh \
            python3 run.py --world_idx $n --out_group ${3}
        # echo $n ${3}
        sleep 2
    done
done
