"""Scenario registry for unified Python / Gazebo experiment environments."""
from scenarios import corner_popout

SCENARIOS = {
    'corner_popout': corner_popout.SCENARIO,
}


def load_scenario(name):
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario '{name}'. Available: {list(SCENARIOS.keys())}")
    return SCENARIOS[name]


def list_scenarios():
    return list(SCENARIOS.keys())
