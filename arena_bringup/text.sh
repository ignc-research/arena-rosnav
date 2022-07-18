maps=`ls ../simulator_setup/maps | grep "map_[[:digit:]][[:digit:]]"`


for map in $maps
do
    echo $map
done

