"""Shared local movement primitive: no network code and no task replay."""
def execute_action(ctrl, step, speed, heading):
    ctrl._check_abort()
    if not ctrl.fw or not ctrl.fw_straight:
        raise RuntimeError('requires V2.3+ firmware')
    if step['t'] == 'F':
        ctrl.drive_straight(step['v'], speed_ms=speed, heading_deg=heading)
    else:
        heading += step['v']
        actual = ctrl.fw_heading()
        if actual is None:
            raise RuntimeError('no heading feedback')
        delta = heading - actual[0]
        if abs(delta) >= 1:
            ctrl.fw_turn(delta)
    return heading
