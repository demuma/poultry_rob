import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple


@dataclass
class Target:
    id: int
    type: str
    priority: int
    x: float
    y: float
    first_seen: float
    last_seen: float
    status: str = "active"
    visited_at: Optional[float] = None


@dataclass
class MissionScoringConfig:
    priority_weight: float
    dwell_weight: float
    distance_weight: float
    stale_weight: float
    max_priority: float
    max_relevant_distance_m: float
    max_dwell_time_sec: float


def available_targets(
    targets: Iterable[Target],
    now: float,
    stale_timeout_sec: float,
    visited_cooldown_sec: float,
    revisit_visited_after_cooldown: bool = True,
) -> List[Target]:
    active = []

    for target in targets:
        age = max(0.0, now - target.last_seen)
        if age > stale_timeout_sec:
            if target.status != "visited":
                target.status = "stale"
            continue

        if target.status == "visited":
            visited_at = target.visited_at or 0.0
            if not revisit_visited_after_cooldown:
                continue
            if now - visited_at < visited_cooldown_sec:
                continue
            target.status = "active"
            target.visited_at = None

        if target.status == "in_progress":
            continue

        active.append(target)

    return active


def score_target(
    target: Target,
    robot_position: Optional[Tuple[float, float]],
    now: float,
    stale_timeout_sec: float,
    config: MissionScoringConfig,
) -> Tuple[float, float]:
    if robot_position is None:
        distance = 0.0
    else:
        distance = math.hypot(target.x - robot_position[0], target.y - robot_position[1])

    max_priority = max(config.max_priority, 1.0)
    max_distance = max(config.max_relevant_distance_m, 0.001)
    max_dwell = max(config.max_dwell_time_sec, 0.001)
    stale_timeout = max(stale_timeout_sec, 0.001)

    normalized_priority = min(float(target.priority) / max_priority, 1.0)
    normalized_distance = min(distance / max_distance, 1.0)
    normalized_dwell = min(max(0.0, now - target.first_seen) / max_dwell, 1.0)
    normalized_age = min(max(0.0, now - target.last_seen) / stale_timeout, 1.0)

    score = (
        config.priority_weight * normalized_priority
        + config.dwell_weight * normalized_dwell
        - config.distance_weight * normalized_distance
        - config.stale_weight * normalized_age
    )

    return score, distance


def select_next_target(
    targets: Iterable[Target],
    robot_position: Optional[Tuple[float, float]],
    now: float,
    stale_timeout_sec: float,
    visited_cooldown_sec: float,
    config: MissionScoringConfig,
    revisit_visited_after_cooldown: bool = True,
):
    candidates = []

    for target in available_targets(
        targets,
        now,
        stale_timeout_sec,
        visited_cooldown_sec,
        revisit_visited_after_cooldown,
    ):
        score, distance = score_target(target, robot_position, now, stale_timeout_sec, config)
        candidates.append((target, score, distance))

    if not candidates:
        return None

    return max(candidates, key=lambda item: item[1])
