#!/bin/bash

for filename in ./build/tests/*; do
    # [ -e "$filename" ] || continue
    extension="${filename##*.}"
    if [[ -x "$filename" ]]; then
        echo "Running $filename"
        $filename
    fi
    echo ""
done

