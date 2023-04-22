#! /usr/bin/env bash

for dir in $(ls -d src/*/)
do
    echo "Updating src/$dir"
    cd $dir
    git pull
    cd -
done
