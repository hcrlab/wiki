# TF

TF is a library for computing transforms between different coordinate frames.
Any node can publish the transform between two coordinate frames at a particular point in time on the `/tf` topic.
A TF listener will listen to the topic and assemble the graph (which must be a tree) of transformations over some period of time (usually 10 seconds).

## Common tasks
### See the frame tree
- `rosrun rqt_tf_tree rqt_tf_tree`
- `rosrun tf view_frames`

## When and when not to use TF
- TF should be used with coordinate frames that can change quickly.
- In general, TF should only be used with permanent coordinate frames (like frames representing the robot itself) that never disappear.
  TF never deletes frame IDs.
  Even if you don't see them in `view_frames`, old frame IDs are stored internally forever.
- NEVER use TF with non-permanent coordinate frames if MoveIt! is running as well.
  MoveIt! will try to transform every single TF frame that has ever been published to the planning frame, even frames that haven't been published in a while.
  As a result, if you stop publishing some frames, MoveIt will spew out error messages saying that it can't transform those frames.
  
For example, if you are using MoveIt! for a pick and place application and you see an object, do not use TF to publish a coordinate frame for the object.
Once the object is taken away and you stop publishing that frame, MoveIt! will freak out.
You can still use TF for inter-robot transforms, but you must implement your own geometry code to transform from a robot frame to the object frame.
