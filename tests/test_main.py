"""DroneHover-Swarm tests — physics, formations, swarm sims, gates.
All assertions are against analytic values or measured sim outcomes
with stated margins."""
import math

import main as ds


# --- per-drone hover physics --------------------------------------------------
def test_hover_thrust_is_mg():
    assert abs(ds.hover_thrust_n(1.6) - 1.6 * ds.G) < 1e-9


def test_disk_area():
    assert abs(ds.disk_area_m2() - 4 * math.pi * 0.12 ** 2) < 1e-12


def test_momentum_power_formula():
    expected = 16.0 ** 1.5 / math.sqrt(2 * ds.RHO_SEA * 0.1810)
    assert abs(ds.ideal_hover_power_w(16.0, ds.RHO_SEA, 0.1810) - expected) < 1e-9


def test_power_scales_with_T_3_2():
    p1 = ds.ideal_hover_power_w(8.0, ds.RHO_SEA, 0.1810)
    p2 = ds.ideal_hover_power_w(16.0, ds.RHO_SEA, 0.1810)
    assert abs(p2 / p1 - 2 ** 1.5) < 1e-9


def test_heavier_needs_more_power():
    assert ds.hover_power_w(2.4) > ds.hover_power_w(1.6)


def test_endurance_bounded():
    assert 10.0 < ds.endurance_min() < 60.0


def test_fleet_power():
    assert abs(ds.fleet_hover_power_w(25) - 25 * ds.hover_power_w()) < 1e-6


# --- formation geometry -------------------------------------------------------
def test_grid_25_centered():
    cen = ds.centroid(ds.formation_slots("grid", 25))
    assert abs(cen[0]) < 1e-12 and abs(cen[1]) < 1e-12


def test_grid_min_pairwise_is_spacing():
    grid = ds.formation_slots("grid", 25)
    assert abs(ds.min_pairwise_m(grid) - ds.DEFAULT_SPACING_M) < 1e-9


def test_grid_spread_max_is_corner():
    spread = ds.spread_metrics(ds.formation_slots("grid", 25))
    assert abs(spread["max_m"] - math.hypot(6.0, 6.0)) < 1e-9


def test_ring_radius_and_chord():
    ring = ds.formation_slots("ring", 25)
    radius = 25 * ds.DEFAULT_SPACING_M / (2.0 * math.pi)
    assert all(abs(math.hypot(x, y) - radius) < 1e-9 for x, y in ring)
    chord = math.dist(ring[0], ring[1])
    assert abs(chord - 2.0 * radius * math.sin(math.pi / 25)) < 1e-9


def test_vee_shape():
    vee = ds.formation_slots("vee", 25)
    assert vee[0] == (0.0, 0.0)
    assert abs(math.dist(vee[0], vee[1]) - ds.DEFAULT_SPACING_M) < 1e-9
    assert vee[1][0] < 0.0 < vee[2][0]


def test_formations_have_n_slots():
    for kind in ("grid", "ring", "vee"):
        assert len(ds.formation_slots(kind, 25)) == 25
        assert len(ds.formation_slots(kind, 10)) == 10


def test_slots_deterministic():
    assert ds.formation_slots("grid", 25) == ds.formation_slots("grid", 25)


# --- distributed stabilization --------------------------------------------------
def test_coupling_reduces_shape_error():
    study = ds.formation_study()
    assert study["coupled"]["max_shape_error_m"] \
        < 0.85 * study["uncoupled"]["max_shape_error_m"]


def test_coupled_shape_error_bounded():
    assert ds.formation_study()["coupled"]["max_shape_error_m"] < 0.30


def test_formation_min_separation_held():
    study = ds.formation_study()
    assert study["coupled"]["min_pairwise_m"] >= ds.MIN_SEPARATION_M


def test_sim_deterministic():
    slots = ds.formation_slots("grid", 25)
    targets = {i: slots[i] for i in range(25)}
    a, _, _ = ds.simulate_swarm(targets, coupling=True)
    b, _, _ = ds.simulate_swarm(targets, coupling=True)
    assert a == b


# --- collision avoidance --------------------------------------------------------
def test_swap_with_avoidance_keeps_separation():
    sw = ds.swap_scenario(avoidance=True)
    assert sw["min_pairwise_m"] >= ds.MIN_SEPARATION_M


def test_swap_with_avoidance_arrives():
    sw = ds.swap_scenario(avoidance=True)
    assert sw["final_error_0_m"] < 0.05 and sw["final_error_1_m"] < 0.05


def test_swap_without_avoidance_collides():
    assert ds.swap_scenario(avoidance=False)["min_pairwise_m"] < 0.30


# --- leader failure ---------------------------------------------------------------
def test_elect_leader_lowest_alive():
    assert ds.elect_leader([7, 3, 12, 5]) == 3


def test_leader_scenario_new_leader():
    assert ds.leader_failure_scenario()["new_leader"] == 1


def test_gap_chain_single_slot_moves():
    lead = ds.leader_failure_scenario()
    assert abs(lead["max_move_m"] - ds.DEFAULT_SPACING_M) < 1e-9
    assert lead["movers"] == len(lead["chain"]) - 1


def test_leader_reform_keeps_separation_and_recovers():
    lead = ds.leader_failure_scenario()
    assert lead["phase_b"]["min_pairwise_m"] >= ds.MIN_SEPARATION_M
    assert lead["recovered"] and lead["phase_b"]["rms_slot_error_m"] < 0.05


# --- comms model ------------------------------------------------------------------
def test_comms_delay_worsens_settling():
    comms = ds.comms_study()
    assert comms["delayed"]["tail_shape_error_m"] \
        > 2.0 * comms["ideal"]["tail_shape_error_m"]


def test_comms_dropout_worsens_further():
    comms = ds.comms_study()
    assert comms["dropped"]["tail_shape_error_m"] \
        > comms["delayed"]["tail_shape_error_m"]


# --- governor + geofence ------------------------------------------------------------
def test_governor_clamps_park():
    gc = ds.governor_clamp("PARK", 15.0, 80.0)
    assert gc["speed_mps"] == 8.0 and gc["alt_m"] == 30.0


def test_governor_absolute_ceiling():
    assert ds.governor_clamp("TEST_AREA", 99.0, 999.0)["alt_m"] \
        == ds.ABSOLUTE_CEILING_M


def test_governor_under_caps_unclamped():
    gc = ds.governor_clamp("PAD", 2.0, 4.0)
    assert not gc["speed_clamped"] and not gc["alt_clamped"]


def test_geofence():
    assert ds.geofence_ok(100.0, 0.0)
    assert not ds.geofence_ok(300.0, 0.0)


# --- interlocks -----------------------------------------------------------------------
def test_interlocks_nominal_pass():
    assert ds.preflight_interlocks(ds.NOMINAL_STATE)["all_pass"]


def test_interlocks_fail_weak_drone():
    state = dict(ds.NOMINAL_STATE, fleet_min_soc=0.20)
    assert not ds.preflight_interlocks(state)["all_pass"]


def test_interlocks_fail_high_wind():
    state = dict(ds.NOMINAL_STATE, wind_mps=14.0)
    assert not ds.preflight_interlocks(state)["all_pass"]


def test_interlocks_fail_missing_gps():
    state = dict(ds.NOMINAL_STATE, gps_locked=23)
    assert not ds.preflight_interlocks(state)["all_pass"]


# --- classifier -------------------------------------------------------------------------
def test_knn_safe():
    assert ds.knn_classify(3.0, 25, 0.90) == "safe"


def test_knn_nogo():
    assert ds.knn_classify(13.0, 40, 0.30) == "no-go"


def test_knn_accuracy():
    assert ds.knn_training_accuracy() >= 0.90


# --- seats -------------------------------------------------------------------------------
def test_carbon4_sim_proceed_live_hold():
    assert ds.carbon4_decide({"live_action": False,
                              "interlocks_pass": True})["verdict"] == "PROCEED"
    assert ds.carbon4_decide({"live_action": True,
                              "interlocks_pass": True})["verdict"] == "HOLD"


def test_carbon4_blocks_failed_interlocks():
    assert ds.carbon4_decide({"live_action": False,
                              "interlocks_pass": False})["verdict"] == "HOLD"


def test_carbon4_trace_deterministic():
    a = ds.carbon4_decide({"live_action": False, "interlocks_pass": True})
    b = ds.carbon4_decide({"live_action": False, "interlocks_pass": True})
    assert a["trace_sha256"] == b["trace_sha256"] and len(a["trace_sha256"]) == 64


def test_muse_never_overrides_hold():
    assert ds.MUSE_ADVISOR["can_override_hold"] is False


def test_flow_plan_only():
    fp = ds.flow_plan()
    assert fp["rendered"] is False and fp["live"] == 0


# --- HOLD gates ------------------------------------------------------------------------------
def test_hold_gates():
    assert len(ds.HOLD_GATES) == 14
    for gate in ("live_flight", "live_swarm_control", "real_hardware_control",
                 "fly_over_people_live", "bypass_geofence",
                 "disable_collision_avoidance", "military_weapons_use",
                 "surveillance_of_people_live", "bvlos_operation_live",
                 "remove_human_override"):
        assert gate in ds.HOLD_GATES


def test_hold_demo_exits_2(capsys):
    assert ds.hold_demo("live_flight") == 2
    assert "HOLD" in capsys.readouterr().out


def test_all_checks_pass():
    checks, passed = ds.run_checks()
    assert passed == len(checks)
