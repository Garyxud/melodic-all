#!/bin/bash

function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }



EXIT_CODE=0

function print_result() {
  if [ $1 -eq 0 ]; then
    echo_green "[Passed]"
  else
    echo_red "[Failed]"
    EXIT_CODE=1
  fi
  echo ""
}


for example in `find examples/ -maxdepth 1 -mindepth 1 -type d`
do
  cd $example
  echo_blue $example
  make -j4 -l4
  print_result $? 
  cd ../..
done

if [ $EXIT_CODE -eq 0 ]; then
  echo_green "All examples built!"
else
  echo_red "There were failed examples"
fi

exit $EXIT_CODE

