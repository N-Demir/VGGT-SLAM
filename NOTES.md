Original code used opencv optical flow to select keyframe images from the initial dataset. I've disabled that, though it's probably a useful general idea. Hopefully that doesn't cause the incremental slam solver to fail? VGGT would be able to handle it fine I think, but this slam implementation uses only a 1 frame overlap to align things...
I think the keyframe selection is a great idea, in section 4.1 the authors say: "utilizing frames with sufficient disparity improves relative depth estimation performance as it adds multi-view information and additionally reduces the number of frames to process." So, the benefits are speed but also accuracy... For my purposes I think I will want a better overall method that sure maybe uses optical flow but has a smaller threshold? And I still want the original images to have poses so it's almost like a two step thing. Get the poses of keyframes then get the poses of the rest based on the keyframe positions. Seems like something the original authors would add though or someone else in the community.

Interesting that it's called slam because it's borderline an offline procedure? Needs some number of frames before that generates a new submap and can then be run through VGGT and aligned with previous submaps. I'm sure it's an easy conversion though to something that's fully online. Just seems almost more like an SfM approach

I added colmap format export, but it takes a long time. Presumably because writing the points takes a long time. 

A lot of points are by default exported (1 for every image pixel, with a percentage filtered out based on confidence threshold). Would be interesting to see the confidence threshold and play around with it... We have way too many points compared to the number of eventual gaussians. Definitely need a smarter way to select the generated number of points. However, these points are highly not geographically spread out... Maybe we could use some simple vector math to better select a subsample (if we save all of them along with their confidences (instead of errors)). Confidence seems directly correlated with image overlap...
Pointcloud confidence thresholding is used for the submap creation and alignment process, so better to downsample the points afterwards I think.

I assume the future of VGGT type models is to output directly gaussians. Because right now the pointcloud outputs really struggle with reflective surfaces.

Also I fix a bug that the generated poses.txt (and for me colmap outputs as well) duplicate the written image positions.
