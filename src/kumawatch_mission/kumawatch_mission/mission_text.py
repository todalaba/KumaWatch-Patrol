"""動画の字幕・HUD・簡潔なログに使う文言(ROS 非依存).

mission_planner が配信する JSON から、審査者が一目で分かる短い英語の文を作る。
- format_event(): /kumawatch/mission_events の 1 件 → ログ 1 行(mission_console が緑で表示)
- status_lines(): /kumawatch/mission_status → HUD の上部に出す 2 行
- bear_panel_lines(): 熊検知時に大きく出す 4 行(BEAR DETECTED / Confidence / Location / Action)
"""


def _point(ev):
    d = ev.get('detail') or {}
    name = d.get('name_en') or ''
    return '%s %s' % (ev.get('point_id', ''), name) if name else ev.get('point_id', '')


def format_event(ev):
    """出来事 1 件を 1 行の英文にする. 表示しない出来事は None."""
    kind = ev.get('event')
    d = ev.get('detail') or {}
    idx, total = ev.get('index') or '', ev.get('total') or ''
    if kind == 'mission_started':
        return 'Mission started: route %s (%s)' % (' -> '.join(d.get('route', [])),
                                                   d.get('strategy', ''))
    if kind == 'navigate':
        risk = d.get('risk')
        return 'Goal %s/%s: %s  (AI risk %.2f)  navigating' % (
            idx, total, _point(ev), risk if risk is not None else float('nan'))
    if kind == 'arrived':
        return 'Goal %s/%s reached: %s  observing' % (idx, total, ev.get('point_id', ''))
    if kind == 'clear':
        return '%s: no bear' % ev.get('point_id', '')
    if kind == 'bear_detected':
        return 'BEAR DETECTED  %s at %s  conf %.2f  %.1f m  -> %s' % (
            d.get('bear_id', ''), d.get('point_id') or '-', d.get('confidence', 0.0),
            d.get('distance_m', 0.0), d.get('action', 'stop'))
    if kind == 'retreat':
        return 'Retreating to safe point (%.1f, %.1f)' % (d.get('x', 0.0), d.get('y', 0.0))
    if kind == 'retreat_done':
        return 'Safe point reached'
    if kind == 'retreat_failed':
        if d.get('remaining'):
            return 'Safe point not reachable, trying another safe point'
        return 'Safe point not reachable'
    if kind == 'safe_hold':
        return 'SAFE HOLD: no reachable safe point, stopped and waiting for the operator'
    if kind == 'operator_command':
        return 'Operator command: %s' % d.get('command', '')
    if kind == 'safe_hold_timeout':
        return 'SAFE HOLD timed out: returning to base'
    if kind in ('unreachable', 'timeout'):
        return 'Goal %s/%s %s: %s' % (idx, total, ev.get('point_id', ''), kind)
    if kind == 'return_to_base':
        return 'All points patrolled: returning to base'
    if kind == 'returned':
        return 'Back at base'
    if kind == 'mission_completed':
        return 'Mission completed: %d/%d points checked, %d bear(s) found, %.1f m, %.0f s' % (
            d.get('num_checked', d.get('num_arrived', 0)), d.get('num_points', 0),
            d.get('bears_found', 0), d.get('distance_m', 0.0), d.get('duration_s', 0.0))
    return None


STATE_TEXT = {
    'RISK_ASSESSMENT': 'AI risk assessment',
    'ROUTE_PLANNING': 'Planning route',
    'WAIT_NAV2': 'Starting navigation',
    'PATROL': 'Navigating',
    'OBSERVE': 'Arrived - observing',
    'EMERGENCY_STOP': 'BEAR - stopped',
    'RETREAT': 'Retreating to safe point',
    'RETURN_TO_BASE': 'Returning to base',
    'FINISHED': 'Mission completed',
}


def status_lines(st):
    """HUD 上部の 2 行: (1 行目) Mission i/N: 地点  (2 行目) Risk / Status / 熊の数・距離."""
    state = st.get('state') or ''
    text = STATE_TEXT.get(state, state or '-')
    if st.get('index') and state in ('PATROL', 'OBSERVE', 'EMERGENCY_STOP', 'RETREAT'):
        line1 = 'Mission %d/%d: %s %s' % (st['index'], st.get('total', 0), st.get('point_id', ''),
                                         st.get('name_en', ''))
        risk = st.get('risk')
        line2 = 'AI risk %.2f  |  %s' % (risk, text) if risk is not None else text
    elif state == 'FINISHED':
        line1 = 'Mission completed'
        line2 = '%d/%d reached' % (st.get('arrived', 0), st.get('total', 0))
    else:
        line1 = 'KumaWatch Patrol'
        line2 = text
    line2 += '  |  bears %d  |  %.1f m' % (st.get('bears_found', 0), st.get('distance_m', 0.0))
    return line1.strip(), line2


def bear_panel_lines(bear):
    """熊検知時のパネル 4 行."""
    return [
        'BEAR DETECTED',
        'Confidence: %.2f   Distance: %.1f m' % (bear.get('confidence', 0.0),
                                                bear.get('distance_m', 0.0)),
        'Location: Patrol point %s' % (bear.get('point_id') or '-'),
        'Action: %s' % bear.get('action', 'Stop and report'),
    ]
