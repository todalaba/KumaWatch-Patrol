import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from kumawatch_mission.frame_pacer import FramePacer  # noqa: E402


def test_regular_frames_are_written_once():
    p = FramePacer(30)
    assert [p.frames_for(i / 30) for i in range(5)] == [1, 1, 1, 1, 1]


def test_sparse_frames_are_repeated_to_keep_real_duration():
    """2 fps でしか届かなくても、10 秒撮れば 30 fps で約 10 秒の動画になる."""
    p = FramePacer(30)
    total = sum(p.frames_for(i * 0.5) for i in range(21))   # 0〜10 秒
    assert total == 301


def test_duplicate_or_old_stamps_are_skipped():
    p = FramePacer(10)
    assert p.frames_for(1.0) == 1
    assert p.frames_for(1.0) == 0
    assert p.frames_for(0.5) == 0
    assert p.frames_for(1.1) == 1


def test_long_gap_is_capped():
    p = FramePacer(10, max_repeat_seconds=2.0)
    p.frames_for(0.0)
    assert p.frames_for(100.0) == 20          # 2 秒分で打ち切り
    assert p.frames_for(100.1) == 1           # その後は通常どおり
