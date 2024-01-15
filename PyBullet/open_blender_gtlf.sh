#! /bin/bash

INPUT_DIR="./rover/meshes"
OUTPUT_DIR="./rover/obj_meshes"
BLENDER="/home/simon/blender/blender-3.6.3-linux-x64/blender"
gltf_files=()

if [ ! -d "$INPUT_DIR" ]; then
  echo "Directory not found: $INPUT_DIR"
  exit 1
fi

if [ ! -d "$OUTPUT_DIR" ]; then
  echo "Directory not found: $OUTPUT_DIR"
  exit 1
fi

for file in "$INPUT_DIR"/*.gltf; do
    if [ -e $file ]; then
        gltf_files+=("$file")
    fi
done

for input_gltf in "${gltf_files[@]}"; do

    $BLENDER -b $input_gltf

    echo "Press any key to continue"
    while [ true ] ; do
        read -n 1 key <&1
        if [[ $key = n ]] ; then
            echo "Next file..."
            break
        fi
    done
done