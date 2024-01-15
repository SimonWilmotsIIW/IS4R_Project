#! /bin/bash

INPUT_DIR="./rover/meshes"
OUTPUT_DIR="./rover/obj_meshes"
BLENDER_DIR="/home/simon/blender/blender-3.6.3-linux-x64/blender"
gltf_files=()

if [ ! -d "$INPUT_DIR" ]; then
  echo "Directory not found: $INPUT_DIR"
  exit 1
fi

if [ ! -d "$OUTPUT_DIR" ]; then
  echo "Directory not found: $OUTPUT_DIR"
  exit 1
fi

# if ! command -v blender &>/dev/null; then
#  echo "Blender is not installed"
#  exit 1
# fi

for file in "$INPUT_DIR"/*.gltf; do
    if [ -e $file ]; then
        gltf_files+=("$file")
    fi
done

for input_gltf in "${gltf_files[@]}"; do
    OUTPUT_OBJ="${input_gltf%.obj}"
    echo ">> Processing $gltf => .obj"
    $BLENDER_DIR -b <<EOF
        import bpy
        import os

        # Clear existing objects
        bpy.ops.wm.read_factory_settings(use_empty=True)

        # Import the .gltf file
        print("$input_gltf")
        bpy.ops.import_scene.gltf(filepath="$input_gltf")

        # Export as .obj
        bpy.ops.export_scene.obj(filepath="$OUTPUT_DIR")

        # Quit Blender
        bpy.ops.wm.quit_blender()
EOF

    echo "Conversion complete: $OUTPUT_OBJ"
    sleep 0.1s
done