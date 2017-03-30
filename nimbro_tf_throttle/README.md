
tf_throttle
-----------

`tf_throttle` creates snapshots of the tf tree at fixed intervals. This reduces
the usually high bandwidth `/tf` topic down to manageable levels.

Subscriptions:
 - `/tf`: Transform input

Publications:
 - `~tf`: Snapshot output. This can be mapped to `/tf` on the receiving machine.

Parameters:
 - `rate` (float): Snapshot rate (default: 4.0)

