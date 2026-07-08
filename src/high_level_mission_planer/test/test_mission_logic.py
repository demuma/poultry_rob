from high_level_mission_planer.mission_logic import (
    MissionScoringConfig,
    Target,
    available_targets,
    score_target,
    select_next_target,
)


def config(**overrides):
    values = {
        "priority_weight": 2.5,
        "dwell_weight": 0.0,
        "distance_weight": 1.0,
        "stale_weight": 0.5,
        "max_priority": 3.0,
        "max_relevant_distance_m": 10.0,
        "max_dwell_time_sec": 60.0,
    }
    values.update(overrides)
    return MissionScoringConfig(**values)


def target(target_id, priority=0, x=0.0, y=0.0, first_seen=90.0, last_seen=100.0, status="active"):
    return Target(
        id=target_id,
        type="HEN",
        priority=priority,
        x=x,
        y=y,
        first_seen=first_seen,
        last_seen=last_seen,
        status=status,
    )


def test_priority_can_beat_shorter_distance():
    far_high_priority = target(1, priority=3, x=8.0, y=0.0)
    near_low_priority = target(2, priority=0, x=1.0, y=0.0)

    selected = select_next_target(
        [far_high_priority, near_low_priority],
        robot_position=(0.0, 0.0),
        now=100.0,
        stale_timeout_sec=5.0,
        visited_cooldown_sec=30.0,
        config=config(dwell_weight=0.0, stale_weight=0.0),
    )

    assert selected is not None
    assert selected[0].id == 1


def test_stale_and_in_progress_targets_are_skipped():
    stale = target(1, last_seen=90.0)
    active = target(2, last_seen=99.0)
    in_progress = target(3, last_seen=99.0, status="in_progress")

    candidates = available_targets(
        [stale, active, in_progress],
        now=100.0,
        stale_timeout_sec=2.0,
        visited_cooldown_sec=30.0,
    )

    assert [candidate.id for candidate in candidates] == [2]
    assert stale.status == "stale"


def test_visited_target_returns_after_cooldown():
    recently_visited = target(1, last_seen=100.0, status="visited")
    recently_visited.visited_at = 95.0

    candidates = available_targets(
        [recently_visited],
        now=100.0,
        stale_timeout_sec=2.0,
        visited_cooldown_sec=30.0,
    )

    assert candidates == []
    assert recently_visited.status == "visited"

    candidates = available_targets(
        [recently_visited],
        now=130.0,
        stale_timeout_sec=40.0,
        visited_cooldown_sec=30.0,
    )

    assert [candidate.id for candidate in candidates] == [1]
    assert recently_visited.status == "active"
    assert recently_visited.visited_at is None


def test_visited_target_can_stay_suppressed_until_it_moves():
    visited = target(1, last_seen=100.0, status="visited")
    visited.visited_at = 10.0

    candidates = available_targets(
        [visited],
        now=130.0,
        stale_timeout_sec=40.0,
        visited_cooldown_sec=30.0,
        revisit_visited_after_cooldown=False,
    )

    assert candidates == []
    assert visited.status == "visited"


def test_selection_can_ignore_visited_target_after_cooldown():
    visited_high_priority = target(1, priority=3, x=0.1, y=0.0, status="visited")
    visited_high_priority.visited_at = 10.0
    active_low_priority = target(2, priority=0, x=5.0, y=0.0)

    selected = select_next_target(
        [visited_high_priority, active_low_priority],
        robot_position=(0.0, 0.0),
        now=130.0,
        stale_timeout_sec=40.0,
        visited_cooldown_sec=30.0,
        config=config(),
        revisit_visited_after_cooldown=False,
    )

    assert selected is not None
    assert selected[0].id == 2


def test_score_rewards_dwell_and_penalizes_age():
    stable = target(1, first_seen=40.0, last_seen=100.0)
    staleish = target(2, first_seen=40.0, last_seen=96.0)

    stable_score, _ = score_target(
        stable,
        robot_position=(0.0, 0.0),
        now=100.0,
        stale_timeout_sec=10.0,
        config=config(priority_weight=0.0, distance_weight=0.0),
    )
    staleish_score, _ = score_target(
        staleish,
        robot_position=(0.0, 0.0),
        now=100.0,
        stale_timeout_sec=10.0,
        config=config(priority_weight=0.0, distance_weight=0.0),
    )

    assert stable_score > staleish_score
