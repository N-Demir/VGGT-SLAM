Original code used opencv optical flow to select keyframe images from the initial dataset. I've disabled that, though it's probably a useful general idea. Hopefully that doesn't cause the incremental slam solver to fail? VGGT would be able to handle it fine I think, but this slam implementation uses only a 1 frame overlap to align things...

I added colmap format export, but it takes a long time. Presumably because writing the points takes a long time. 

A lot of points are by default exported (1 for every image pixel, with a percentage filtered out based on confidence threshold). Would be interesting to see the confidence threshold and play around with it... We have way too many points compared to the number of eventual gaussians... 

I assume the future of VGGT type models is to output directly gaussians. Because right now the pointcloud outputs really struggle with reflective surfaces.

Also I fix a bug that the generated poses.txt (and for me colmap outputs as well) duplicate the written image positions.
