Navigation server roadmap

Navigation server is split into a few different classes of behavior based on
 the complexity of the behavior and instruments used in feedback.

The simplest beaviors end in "change". These quantities are directly
measured and controlled, as are expected to be most rodust.

heading_change
depth_change

DVL based behaviors are more complex as they will probably require a sensor
fussion with a compass. The DVL is capable of measuring rotations, but the
quality of the position estimate suffers in these cases. Becuase of this, DVL
based motion will attempt to move with as little rotation as possible.

towaypoint_DVL

Vision based behaviors are prehaps the most comlicated, because it is common to
combine them with other motions.

centeron_vision
linefollow_vision

Feedback interrupt behaviors are used stop a behavior when an event occurs.

stoponobject_feedback

Motion injection feedbacks are used to keep the vehicle moving as it is
 preforming another task.

addvelocity_feedback

