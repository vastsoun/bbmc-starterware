#!/bin/bash 

DEST="/home/vassilis/Dropbox/MyStuff/thesis/dev-sw/starterware/bmbb-1/"

for folder in apps drivers include contrlib system_config platform utils platform ;
do
    cp -u -r $folder $DEST
done

