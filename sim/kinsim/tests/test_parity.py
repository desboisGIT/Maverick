"""kinsim repeats the ROS nodes' scheduling glue: fail when the originals change."""

from kinsim import parity


def test_node_glue_unchanged_since_kinsim_copied_it():
    bad = parity.diff()
    assert not bad, (
        "the ROS node code kinsim re-implements changed: "
        + ", ".join(bad)
        + ". Copy the change into kinsim/stack.py (or memory.py), then "
        "`python3 -m kinsim.parity --update` (docs/kinsim.md, 'Keeping kinsim in sync')."
    )
