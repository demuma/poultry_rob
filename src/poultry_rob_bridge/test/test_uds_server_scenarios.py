from poultry_rob_bridge.scenario_uds_server import build_frame, priority_ramp
from poultry_rob_bridge.uds_server import build_frame as build_minimal_frame


def objects_by_id(frame):
    return {obj.id: obj for obj in frame.objects}


def test_priority_ramp_steps_every_15_seconds():
    assert priority_ramp(0.0) == 0
    assert priority_ramp(14.9) == 0
    assert priority_ramp(15.0) == 1
    assert priority_ramp(30.0) == 2
    assert priority_ramp(45.0) == 3
    assert priority_ramp(90.0) == 3


def test_minimal_uds_server_matches_dil_shape_without_scenario_logic():
    frame = build_minimal_frame(1)

    assert frame.header.seq == 1
    assert frame.header.frame_id == "camera_optical_frame"
    assert sorted(objects_by_id(frame)) == [1, 2]
    assert all(obj.type == "HEN" for obj in frame.objects)


def test_new_near_hen_adds_third_hen_after_three_seconds():
    early = objects_by_id(build_frame(1, 2.9, "new_near_hen"))
    late = objects_by_id(build_frame(2, 3.0, "new_near_hen"))

    assert sorted(early) == [1, 2]
    assert sorted(late) == [1, 2, 3]
    assert late[3].priority == 0


def test_high_priority_far_hen_appears_later():
    early = objects_by_id(build_frame(1, 2.9, "new_high_priority_far"))
    late = objects_by_id(build_frame(2, 3.0, "new_high_priority_far"))

    assert sorted(early) == [1]
    assert sorted(late) == [1, 2]
    assert late[2].priority == 3


def test_hen_disappears_before_arrival():
    early = objects_by_id(build_frame(1, 3.9, "hen_disappears_before_arrival"))
    late = objects_by_id(build_frame(2, 4.0, "hen_disappears_before_arrival"))

    assert sorted(early) == [1, 2]
    assert sorted(late) == [2]


def test_hen_moves_keeps_id_and_updates_position():
    early = objects_by_id(build_frame(1, 2.9, "hen_moves"))
    middle = objects_by_id(build_frame(2, 3.0, "hen_moves"))
    late = objects_by_id(build_frame(3, 6.0, "hen_moves"))

    assert early[1].position.x == 5.0
    assert middle[1].position.x == 4.0
    assert late[1].position.x == 3.0


def test_many_hens_uniform_is_deterministic_and_large():
    first = build_frame(1, 0.0, "many_hens_uniform")
    second = build_frame(2, 10.0, "many_hens_uniform")

    assert len(first.objects) == 120
    assert len(second.objects) == 120
    assert first.objects[0].position.x == second.objects[0].position.x
    assert first.objects[0].position.y == second.objects[0].position.y
    assert all(0 <= obj.priority <= 3 for obj in first.objects)


def test_many_hens_clusters_and_hotspot_are_large():
    clustered = build_frame(1, 0.0, "many_hens_clusters")
    hotspot = build_frame(1, 0.0, "many_hens_hotspot")

    assert len(clustered.objects) == 160
    assert len(hotspot.objects) == 180
    assert any(obj.priority == 3 for obj in hotspot.objects)


def test_visit_event_removes_hen_near_goal_position():
    frame = build_frame(
        1,
        0.0,
        "basic",
        visit_events=[(6.1, -1.1)],
        visit_clear_radius_m=0.8,
    )

    assert sorted(objects_by_id(frame)) == [2]


def test_visit_event_can_remove_multiple_clustered_hens():
    frame = build_frame(
        1,
        3.0,
        "new_near_hen",
        visit_events=[(2.1, -0.8)],
        visit_clear_radius_m=0.8,
    )

    assert sorted(objects_by_id(frame)) == [1, 2]
