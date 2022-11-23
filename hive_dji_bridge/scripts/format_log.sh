#!/bin/bash

echo "checking if log file $1 exists"

if [ -f "$1" ]; then
  echo "correct"
else
  echo "file $1 doesn't exist"
  exit
fi

EXP="\[topics: (.*?\])"

if [[ $(cat $1) =~ $EXP ]]; then
  echo "formatting file"
  echo "$(perl -pe 's/(\/\w+) //g; s/\[topics: .*?\] //g; s/\[\/.*?\.(h|c)pp:\d+//g; s/(\(.*?\)\])//g; s/(\d{10}).(\d{9})/$1/g' $1)" > $1
  echo "$(perl -MPOSIX -pe 's/(\d+) /strftime "%F %T ", localtime($1)/se' $1)" > $1
else
  echo "file is already formated"
fi

$EDITOR $1
