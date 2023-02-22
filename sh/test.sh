#!/bin/bash

while getopts ":dv" opt; do
  case $opt in
    d)
      draw=true
      ;;
    v)
      verify=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

if [ "$draw" = true ]; then
  echo "Drawing..."
fi

if [ "$verify" = true ]; then
  echo "Verifying..."
fi
