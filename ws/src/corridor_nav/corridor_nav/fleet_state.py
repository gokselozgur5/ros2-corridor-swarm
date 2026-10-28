"""One fleet view: corridor_msgs/FleetState on /fleet/state every `period_s` (ADR-0015).

Per vehicle (parameter `vehicles`) it reads /<v>/fix, /<v>/imu, /<v>/odom, /<v>/follower/status and the
patrol action's status topic, plus /fleet/geofence_events, and fills one VehicleState: position from the
fix through the corridor LocalFrame (never odom x/y, #19), heading from the IMU, speed from odom,
inside_corridor from the polygon, mode and goal_id derived from what the vehicle layer reports. Nothing
here is told by the planner; a vehicle driven by an operator tool shows the same as one driven by Claude.
A vehicle whose fix is older than `fix_timeout_s` is still listed, with `mode` 'unknown' and no fix.
"""
from action_msgs.msg import GoalStatus, GoalStatusArray


ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


def active_goal_id(status_array):
    """Hex id of the goal the action server is running, '' when none is accepted, executing or canceling."""
    for st in status_array.status_list:
        if st.status in ACTIVE:
            return bytes(st.goal_info.goal_id.uuid).hex()
    return ''


def mode_of(patrol_goal_id, follower_status, breached):
    """idle | patrol | goto | geofence_hold | unknown from what the vehicle layer reports.

    A geofence EXIT without an ENTER since holds the vehicle whatever else is going on; a running patrol
    goal is 'patrol'; a follower that follows a route nobody's patrol owns is a goto (planner, clicked
    goal, send_path alike); otherwise idle. 'unknown' only when the follower never reported."""
    if breached:
        return 'geofence_hold'
    if patrol_goal_id:
        return 'patrol'
    if follower_status == 'following':
        return 'goto'
    if follower_status in ('idle', 'arrived'):
        return 'idle'
    return 'unknown'


