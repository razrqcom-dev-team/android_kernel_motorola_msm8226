#!/bin/bash
 
best=9999999999
tag=""
 
for commit in $(git tag -l "*8x26*")
do
    match=$(git diff $commit --shortstat --diff-filter M | awk -F' ' '{print $4 + $6}')
    echo $commit has $match lines changed
 
    if [ $match -lt $best ]; then
        best=$match
        tag=$commit
        echo $tag is the new best match with $best lines changed
    fi
done
 
echo "Best match"
echo TAG : $tag
echo Lines Changed : $best
