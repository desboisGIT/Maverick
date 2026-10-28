import math

from maverick_skills.search import clear_goal


def test_goal_far_from_people_is_unchanged():
    assert clear_goal((4.5, 3.9), [(1.0, 1.0)], lambda p: True, 1.2) == (4.5, 3.9)
    assert clear_goal((4.5, 3.9), [], lambda p: True, 1.2) == (4.5, 3.9)


def test_goal_next_to_a_person_is_moved_to_a_free_point_clear_of_them():
    person = (4.5, 3.1)  # 0.8 m from the goal
    g = clear_goal((4.5, 3.9), [person], lambda p: True, 1.2)
    assert math.dist(g, person) >= 1.2 and math.dist(g, (4.5, 3.9)) <= 1.0


def test_only_free_points_are_chosen_and_failure_returns_the_goal():
    person = (4.5, 3.1)
    g = clear_goal((4.5, 3.9), [person], lambda p: p[1] > 3.9, 1.2)
    assert g[1] > 3.9 and math.dist(g, person) >= 1.2
    assert clear_goal((4.5, 3.9), [person], lambda p: False, 1.2) == (4.5, 3.9)


def test_stamp_people_marks_a_disc_and_keeps_the_input():
    import numpy as np
    from maverick_mapping.voxel_map import FREE, OCCUPIED
    from maverick_skills.navigation import stamp_people

    g = np.full((60, 60), FREE, dtype=np.int8)
    g2, res, org = stamp_people((g, 0.1, (0.0, 0.0)), [(3.0, 3.0)], 1.0)
    assert (g == FREE).all()  # untouched
    assert g2[30, 30] == OCCUPIED and g2[30, 39] == OCCUPIED and g2[30, 42] == FREE
    assert stamp_people((g, 0.1, (0.0, 0.0)), [], 1.0)[0] is g
    assert stamp_people(None, [(1, 1)], 1.0) is None
