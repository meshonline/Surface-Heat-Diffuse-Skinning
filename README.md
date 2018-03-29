# Surface-Heat-Diffuse-Skinning
Experiment of prototype concepts.
# Introduction
Blender already has built-in bone heat weighting, it works well in most time, but sometimes it fails to find a solution.

This projet is an experiment of prototype concepts, I try to use iterations to approach a stable solution.

The source code of the octree part is not optimized, it runs quite slow, but the source code is easy to read and understand.

# Build
There is a 'Readme.txt' in the 'src' sub-directory.

# Install
Build the project from source code, then copy the binary to corresponding directories:

'Surface-Heat-Diffuse-Skinning/addon/surface_heat_diffuse_skinning/bin/Windows': Windows(64bit).

'Surface-Heat-Diffuse-Skinning/addon/surface_heat_diffuse_skinning/bin/Linux': Linux(64bit).

'Surface-Heat-Diffuse-Skinning/addon/surface_heat_diffuse_skinning/bin/Darwin': macOS(64bit).

Copy all the contents in 'Surface-Heat-Diffuse-Skinning/addon' to Blender's add-ons directory.

Launch Blender, from 'File->User Preferences...', active the 'Surface Heat Diffuse Skinning' add-on, click the 'Save User Settings' button.

# Usage
When you select the meshes and the armature, the interface will appear in the 'Animation' tab of the tools shelf on the left.

# Similar project
http://www.mesh-online.net/voxel.html
