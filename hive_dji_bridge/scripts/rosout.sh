#!/bin/bash

# show log in runtime
tail -f ~/.ros/log/latest/rosout.log | perl -pe 's/\[\/.*?\.(h|c)pp:\d+(\(.*?\)) \[topics: (.*?\]) //g; s/^(\d{10}).(\d{9})/$1/g'
