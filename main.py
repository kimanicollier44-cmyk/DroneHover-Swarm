#!/usr/bin/env python3
"""
DroneHover-Swarm — Hover Drone Swarm Coordination Lab
======================================================
by Darion Kellar

SIM-ONLY educational lab for a coordinated swarm of small hover
drones — HoverX scaled from ONE vehicle to a 25-drone formation.
Pure Python stdlib, ONE main.py, zero network, zero dependencies.
Nothing here controls real hardware, flies a real drone, or leaves
the sandbox — every live action is behind a HOLD gate, and the
HOLD always wins.

What is modelled, with real physics / control math:
  1. Per-drone hover model .... momentum-theory power, battery endurance
  2. Formation geometry ....... grid / ring / V slots, centroid, spread
  3. Distributed stabilization  spring-damper neighbour coupling (virtual
                                physics) vs independent PD under gusts
  4. Collision avoidance ...... damped potential-field repulsion + swirl;
                                minimum separation is never violated
  5. Leader failure ........... lowest-id re-election + gap-closing chain:
                                each neighbour shifts exactly one slot
  6. Comms model .............. message delay / dropout degrades settling
  7. Swarm governor ........... zone caps + geofence (X-family style)
  8. Pre-flight interlocks .... fleet battery, GPS, wind, mesh, clear zone
  9. Regime classifier ........ stdlib kNN: safe / marginal / no-go
 10. Advisory seats ........... Carbon 4 decider (tighten-only), Muse
                                advisor (never overrides a HOLD), Flow
                                plan-only (rendered=False, live=0)

The sim works in the horizontal plane at a fixed, governor-approved
altitude (30 m, the PARK zone cap). All runs are deterministic —
double-run byte-identical.

Run:
  python3 main.py            full demo + CHECKS, prints VERDICT
  python3 main.py demo       same as above
  python3 main.py hold-demo  ask for a live flight -> HOLD, exit 2
"""

import hashlib
import math
import sys
from collections import deque

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
G = 9.80665            # standard gravity, m/s^2
RHO_SEA = 1.225        # air density at sea level, kg/m^3

# ---------------------------------------------------------------------------
# 1. Per-drone hover model — a small quad, HoverX physics at small scale
#    Ideal induced power (momentum theory): P = T^(3/2) / sqrt(2 * rho * A)
#    Real electric power divides by the figure of merit.
# ---------------------------------------------------------------------------
DRONE = {
    "name": "SwarmQuad (sim)",
    "mass_kg": 1.6,               # small quadcopter, all-up
    "rotors": 4,
    "rotor_radius_m": 0.12,       # per rotor
    "figure_of_merit": 0.68,      # small-rotor figure of merit
    "battery_wh": 68.0,
    "usable_soc_frac": 0.80,      # only 80% of pack is usable in sim
}
DEFAULT_SWARM_SIZE = 25
FORMATION_ALT_M = 30.0            # PARK-zone cap, see governor below


def hover_thrust_n(mass_kg):
    """Thrust needed to hover: T = m * g. Exact by definition."""
    return mass_kg * G


def disk_area_m2():
    """Total rotor disk area across all rotors of one drone."""
    return DRONE["rotors"] * math.pi * DRONE["rotor_radius_m"] ** 2


def ideal_hover_power_w(thrust_n, rho=RHO_SEA, area_m2=None):
    if area_m2 is None:
        area_m2 = disk_area_m2()
    return thrust_n ** 1.5 / math.sqrt(2.0 * rho * area_m2)


def hover_power_w(mass_kg=DRONE["mass_kg"], rho=RHO_SEA):
    """Real hover power = ideal momentum power / figure of merit."""
    return ideal_hover_power_w(hover_thrust_n(mass_kg), rho) / DRONE["figure_of_merit"]


def endurance_min(mass_kg=DRONE["mass_kg"], soc_frac=1.0):
    """Hover endurance on the usable pack at a given state of charge."""
    usable_wh = DRONE["battery_wh"] * DRONE["usable_soc_frac"] * soc_frac
    return usable_wh / hover_power_w(mass_kg) * 60.0


def fleet_hover_power_w(n=DEFAULT_SWARM_SIZE):
    """Whole-swarm hover power: n identical drones."""
    return n * hover_power_w()


# ---------------------------------------------------------------------------
# 2. Formation geometry — deterministic slot assignment in the plane
# ---------------------------------------------------------------------------
DEFAULT_SPACING_M = 3.0
MIN_SEPARATION_M = 1.5            # hard floor: drones never come closer


def formation_slots(kind, n=DEFAULT_SWARM_SIZE, spacing=DEFAULT_SPACING_M):
    """Slot (x, y) for each drone index, centred on the origin."""
    if kind == "grid":
        side = math.ceil(math.sqrt(n))
        rows = math.ceil(n / side)
        out = []
        for idx in range(n):
            r, c = divmod(idx, side)
            out.append(((c - (side - 1) / 2.0) * spacing,
                        ((rows - 1) / 2.0 - r) * spacing))
        return out
    if kind == "ring":
        radius = n * spacing / (2.0 * math.pi)
        return [(radius * math.cos(2.0 * math.pi * i / n),
                 radius * math.sin(2.0 * math.pi * i / n)) for i in range(n)]
    if kind == "vee":
        out = [(0.0, 0.0)]                       # leader at the front
        leg = spacing / math.sqrt(2.0)           # 45-degree legs
        for i in range(1, n):
            rank = (i + 1) // 2
            side = -1.0 if i % 2 == 1 else 1.0   # alternate left / right
            out.append((side * rank * leg, -rank * leg))
        return out
    raise ValueError(f"unknown formation: {kind}")


def centroid(positions):
    n = len(positions)
    return (sum(p[0] for p in positions) / n,
            sum(p[1] for p in positions) / n)


def spread_metrics(positions):
    """Mean / max distance to the centroid — how wide the formation is."""
    c = centroid(positions)
    dists = [math.dist(p, c) for p in positions]
    return {"mean_m": sum(dists) / len(dists), "max_m": max(dists)}


def min_pairwise_m(positions):
    best = float("inf")
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            best = min(best, math.dist(positions[i], positions[j]))
    return best


def neighbor_pairs(slots, spacing=DEFAULT_SPACING_M):
    """Slot pairs close enough to be coupled neighbours (<= 1.01 x spacing)."""
    pairs = []
    for i in range(len(slots)):
        for j in range(i + 1, len(slots)):
            if math.dist(slots[i], slots[j]) <= 1.01 * spacing:
                pairs.append((i, j))
    return pairs


# ---------------------------------------------------------------------------
# 3-6. Swarm simulation — PD slot hold + spring-damper neighbour coupling
#      + damped potential-field collision avoidance, under a gust field.
#      Neighbour state can be delayed / dropped by the comms model.
# ---------------------------------------------------------------------------
KP, KD = 2.0, 2.4                 # slot PD gains
KC, KCD = 3.0, 0.8                # neighbour spring / damper gains
REPEL_RANGE_M = 2.4               # avoidance wakes inside this distance
REPEL_K = 25.0                    # repulsion spring
REPEL_C = 6.0                     # repulsion damping on closing speed
REPEL_SWIRL = 0.7                 # tangential swirl: breaks head-on symmetry
SIM_DT = 0.02


def gust_accel(t, x, y, amp=1.6):
    """Deterministic gust field (m/s^2): a smooth pulse, t in [2, 7] s,
    varying with position so neighbours are pushed differently."""
    if t < 2.0 or t > 7.0:
        return (0.0, 0.0)
    env = math.sin(math.pi * (t - 2.0) / 5.0)
    return (amp * math.sin(1.3 * t + 0.30 * x) * env,
            amp * math.cos(1.05 * t + 0.27 * y) * env)


def _shape_error(pos_list, slot_list, pairs):
    s = 0.0
    for a, b in pairs:
        dx = (pos_list[b][0] - pos_list[a][0]) - (slot_list[b][0] - slot_list[a][0])
        dy = (pos_list[b][1] - pos_list[a][1]) - (slot_list[b][1] - slot_list[a][1])
        s += dx * dx + dy * dy
    return math.sqrt(s / max(len(pairs), 1))


def simulate_swarm(targets, pos0=None, vel0=None, seconds=10.0,
                   coupling=True, avoidance=True, delay_steps=0,
                   dropout_mod=0, gust_amp=1.6, t0=0.0):
    """Run the swarm. targets: {drone_id: slot}. Returns (metrics, pos, vel).

    Comms model (labelled model, not a radio simulation): with
    delay_steps > 0 each drone sees neighbour states that many steps
    old; with dropout_mod > 0 a deterministic 1-in-N update pattern
    falls back to a state twice as stale again.
    """
    ids = sorted(targets)
    slot_list = [targets[i] for i in ids]
    pairs = neighbor_pairs(slot_list)
    nbrs = {i: [] for i in ids}
    for a, b in pairs:
        nbrs[ids[a]].append(ids[b])
        nbrs[ids[b]].append(ids[a])
    pos = {i: list(pos0[i]) if pos0 else list(targets[i]) for i in ids}
    vel = {i: list(vel0[i]) if vel0 else [0.0, 0.0] for i in ids}
    history = [({i: tuple(pos[i]) for i in ids},
                {i: tuple(vel[i]) for i in ids})]
    steps = int(seconds / SIM_DT)
    max_shape = 0.0
    min_dist = float("inf")
    tail_errors = []
    for s in range(steps):
        t = t0 + s * SIM_DT

        def view(k):
            lag = delay_steps
            if dropout_mod and (s + 3 * k) % dropout_mod == 0:
                lag = delay_steps * 2 + 5
            snap = history[max(0, len(history) - 1 - lag)]
            return snap[0][k], snap[1][k]

        acc = {}
        for i in ids:
            ax = KP * (targets[i][0] - pos[i][0]) - KD * vel[i][0]
            ay = KP * (targets[i][1] - pos[i][1]) - KD * vel[i][1]
            if coupling:
                for j in nbrs[i]:
                    pj, vj = view(j)
                    ax += KC * ((pj[0] - pos[i][0]) - (targets[j][0] - targets[i][0])) \
                        + KCD * (vj[0] - vel[i][0])
                    ay += KC * ((pj[1] - pos[i][1]) - (targets[j][1] - targets[i][1])) \
                        + KCD * (vj[1] - vel[i][1])
            gx, gy = gust_accel(t, pos[i][0], pos[i][1], amp=gust_amp)
            acc[i] = [ax + gx, ay + gy]
        if avoidance:
            for a_i in range(len(ids)):
                for b_i in range(a_i + 1, len(ids)):
                    i, j = ids[a_i], ids[b_i]
                    dx, dy = pos[i][0] - pos[j][0], pos[i][1] - pos[j][1]
                    d = math.hypot(dx, dy)
                    if 1e-9 < d < REPEL_RANGE_M:
                        ux, uy = dx / d, dy / d
                        for a, b, sx, sy in ((i, j, ux, uy), (j, i, -ux, -uy)):
                            closing = -((vel[a][0] - vel[b][0]) * sx
                                        + (vel[a][1] - vel[b][1]) * sy)
                            mag = REPEL_K * (REPEL_RANGE_M - d) \
                                + REPEL_C * max(0.0, closing)
                            acc[a][0] += mag * (sx - REPEL_SWIRL * sy)
                            acc[a][1] += mag * (sy + REPEL_SWIRL * sx)
        for i in ids:
            vel[i][0] += acc[i][0] * SIM_DT
            vel[i][1] += acc[i][1] * SIM_DT
            pos[i][0] += vel[i][0] * SIM_DT
            pos[i][1] += vel[i][1] * SIM_DT
        history.append(({i: tuple(pos[i]) for i in ids},
                        {i: tuple(vel[i]) for i in ids}))
        pos_list = [pos[i] for i in ids]
        err = _shape_error(pos_list, slot_list, pairs)
        max_shape = max(max_shape, err)
        min_dist = min(min_dist, min_pairwise_m(pos_list))
        if t - t0 >= seconds - 1.0:
            tail_errors.append(err)
    pos_list = [pos[i] for i in ids]
    rms_slot = math.sqrt(sum((pos[i][0] - targets[i][0]) ** 2
                             + (pos[i][1] - targets[i][1]) ** 2
                             for i in ids) / len(ids))
    metrics = {
        "max_shape_error_m": max_shape,
        "tail_shape_error_m": sum(tail_errors) / max(len(tail_errors), 1),
        "final_shape_error_m": _shape_error(pos_list, slot_list, pairs),
        "min_pairwise_m": min_dist,
        "rms_slot_error_m": rms_slot,
    }
    return metrics, pos, vel


def swap_scenario(avoidance=True, seconds=8.0):
    """Two drones trade places head-on along the same line. With the
    avoidance field on, the swirl walks them around each other; with
    it off, they pass through each other (a sim near-collision)."""
    targets = {0: (6.0, 0.0), 1: (-6.0, 0.0)}
    pos = {0: [-6.0, 0.0], 1: [6.0, 0.0]}
    vel = {0: [0.0, 0.0], 1: [0.0, 0.0]}
    steps = int(seconds / SIM_DT)
    min_dist = float("inf")
    for _ in range(steps):
        acc = {}
        for i in (0, 1):
            acc[i] = [KP * (targets[i][0] - pos[i][0]) - KD * vel[i][0],
                      KP * (targets[i][1] - pos[i][1]) - KD * vel[i][1]]
        if avoidance:
            dx, dy = pos[0][0] - pos[1][0], pos[0][1] - pos[1][1]
            d = math.hypot(dx, dy)
            if 1e-9 < d < REPEL_RANGE_M:
                ux, uy = dx / d, dy / d
                for a, b, sx, sy in ((0, 1, ux, uy), (1, 0, -ux, -uy)):
                    closing = -((vel[a][0] - vel[b][0]) * sx
                                + (vel[a][1] - vel[b][1]) * sy)
                    mag = REPEL_K * (REPEL_RANGE_M - d) \
                        + REPEL_C * max(0.0, closing)
                    acc[a][0] += mag * (sx - REPEL_SWIRL * sy)
                    acc[a][1] += mag * (sy + REPEL_SWIRL * sx)
        for i in (0, 1):
            vel[i][0] += acc[i][0] * SIM_DT
            vel[i][1] += acc[i][1] * SIM_DT
            pos[i][0] += vel[i][0] * SIM_DT
            pos[i][1] += vel[i][1] * SIM_DT
        min_dist = min(min_dist, math.dist(pos[0], pos[1]))
    return {
        "min_pairwise_m": min_dist,
        "final_error_0_m": math.dist(pos[0], targets[0]),
        "final_error_1_m": math.dist(pos[1], targets[1]),
    }


# ---------------------------------------------------------------------------
# 5. Leader failure — deterministic re-election + gap-closing chain
# ---------------------------------------------------------------------------
def elect_leader(alive_ids):
    """Deterministic rule: the lowest surviving drone id leads."""
    return min(alive_ids)


def _slot_adjacency(slots):
    adj = {i: [] for i in range(len(slots))}
    for i, j in neighbor_pairs(slots):
        adj[i].append(j)
        adj[j].append(i)
    for i in adj:
        adj[i].sort()
    return adj


def close_gap_chain(slots, hole_index):
    """Shortest slot-graph path from the hole to the last slot. Every
    drone on the path shifts exactly one slot toward the hole, so the
    gap walks to the formation's edge instead of one drone crossing
    the whole formation. Returns the path [hole, ..., last]."""
    adj = _slot_adjacency(slots)
    target = len(slots) - 1
    prev = {hole_index: None}
    queue = deque([hole_index])
    while queue:
        cur = queue.popleft()
        if cur == target:
            break
        for nb in adj[cur]:
            if nb not in prev:
                prev[nb] = cur
                queue.append(nb)
    path = []
    cur = target
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


_SCENARIO_CACHE = {}


def formation_study():
    """25-drone grid in the gust field: coupled vs independent PD."""
    if "formation" not in _SCENARIO_CACHE:
        slots = formation_slots("grid", DEFAULT_SWARM_SIZE)
        targets = {i: slots[i] for i in range(len(slots))}
        coupled, _, _ = simulate_swarm(targets, coupling=True)
        uncoupled, _, _ = simulate_swarm(targets, coupling=False)
        _SCENARIO_CACHE["formation"] = {"coupled": coupled,
                                        "uncoupled": uncoupled}
    return _SCENARIO_CACHE["formation"]


def comms_study():
    """Same gust run, neighbour data ideal vs delayed vs delayed+dropped.
    Settling quality is read from the last second of the run."""
    if "comms" not in _SCENARIO_CACHE:
        slots = formation_slots("grid", DEFAULT_SWARM_SIZE)
        targets = {i: slots[i] for i in range(len(slots))}
        ideal = formation_study()["coupled"]
        delayed, _, _ = simulate_swarm(targets, coupling=True, delay_steps=15)
        dropped, _, _ = simulate_swarm(targets, coupling=True,
                                       delay_steps=15, dropout_mod=5)
        _SCENARIO_CACHE["comms"] = {"ideal": ideal, "delayed": delayed,
                                    "dropped": dropped}
    return _SCENARIO_CACHE["comms"]


def leader_failure_scenario():
    """Drone 0 (leader, corner slot) drops out mid-gust. The swarm
    re-elects drone 1, closes the gap with a one-slot cascade, and
    re-forms. Phase B flies in calm air so recovery is measurable."""
    if "leader" not in _SCENARIO_CACHE:
        slots = formation_slots("grid", DEFAULT_SWARM_SIZE)
        targets = {i: slots[i] for i in range(len(slots))}
        _, pos_a, vel_a = simulate_swarm(targets, seconds=4.0)
        alive = [i for i in targets if i != 0]
        new_leader = elect_leader(alive)
        chain = close_gap_chain(slots, 0)
        new_targets = {i: slots[i] for i in alive}
        for k in range(len(chain) - 1):
            new_targets[chain[k + 1]] = slots[chain[k]]
        max_move = max(math.dist(slots[i], new_targets[i]) for i in alive)
        metrics_b, _, _ = simulate_swarm(new_targets, pos0=pos_a, vel0=vel_a,
                                         seconds=6.0, gust_amp=0.0, t0=4.0)
        _SCENARIO_CACHE["leader"] = {
            "new_leader": new_leader,
            "chain": chain,
            "movers": sum(1 for i in alive
                          if math.dist(slots[i], new_targets[i]) > 0.01),
            "max_move_m": max_move,
            "phase_b": metrics_b,
            "recovered": metrics_b["rms_slot_error_m"] < 0.05,
        }
    return _SCENARIO_CACHE["leader"]


# ---------------------------------------------------------------------------
# 7. Swarm governor — X-family style caps by zone + geofence
# ---------------------------------------------------------------------------
GOVERNOR_ZONES = [
    {"zone": "PAD",       "max_speed_mps": 3.0,  "max_alt_m": 5.0},
    {"zone": "PARK",      "max_speed_mps": 8.0,  "max_alt_m": 30.0},
    {"zone": "FIELD",     "max_speed_mps": 12.0, "max_alt_m": 60.0},
    {"zone": "TEST_AREA", "max_speed_mps": 20.0, "max_alt_m": 120.0},
]
ABSOLUTE_CEILING_M = 120.0
GEOFENCE_RADIUS_M = 250.0         # from home, in the sim plane


def governor_clamp(zone_name, want_speed_mps, want_alt_m):
    zone = next(z for z in GOVERNOR_ZONES if z["zone"] == zone_name)
    speed = min(want_speed_mps, zone["max_speed_mps"])
    alt = min(want_alt_m, zone["max_alt_m"], ABSOLUTE_CEILING_M)
    return {
        "zone": zone_name,
        "speed_mps": speed,
        "alt_m": alt,
        "speed_clamped": speed < want_speed_mps,
        "alt_clamped": alt < want_alt_m,
    }


def geofence_ok(x, y):
    """True while the point stays inside the geofence radius of home."""
    return math.hypot(x, y) <= GEOFENCE_RADIUS_M


# ---------------------------------------------------------------------------
# 8. Pre-flight interlocks — fleet-wide: ALL must pass before sim takeoff
# ---------------------------------------------------------------------------
MAX_SWARM_SIZE = 50


def preflight_interlocks(state):
    """state keys: fleet_min_soc, fleet_avg_soc, gps_locked, swarm_size,
    wind_mps, clear_zone, comms_ok_frac, geofence_loaded, governor_set"""
    checks = [
        ("fleet_battery_min", state["fleet_min_soc"] >= 0.35,
         f"weakest drone SoC {state['fleet_min_soc']:.0%} >= 35%"),
        ("fleet_battery_avg", state["fleet_avg_soc"] >= 0.60,
         f"fleet average SoC {state['fleet_avg_soc']:.0%} >= 60%"),
        ("gps_lock_all", state["gps_locked"] >= state["swarm_size"],
         f"GPS lock {state['gps_locked']}/{state['swarm_size']} drones"),
        ("wind_limit", state["wind_mps"] <= 10.0,
         f"wind {state['wind_mps']:.1f} m/s <= 10.0 m/s"),
        ("clear_zone", bool(state["clear_zone"]),
         "launch / landing zone clear"),
        ("comms_mesh", state["comms_ok_frac"] >= 0.90,
         f"mesh check-in {state['comms_ok_frac']:.0%} >= 90% of swarm"),
        ("geofence_loaded", bool(state["geofence_loaded"]),
         "geofence + governor caps loaded on every drone"),
        ("swarm_size", 2 <= state["swarm_size"] <= MAX_SWARM_SIZE,
         f"swarm size {state['swarm_size']} within 2..{MAX_SWARM_SIZE}"),
    ]
    return {
        "all_pass": all(ok for _, ok, _ in checks),
        "checks": [{"name": n, "ok": ok, "detail": d} for n, ok, d in checks],
    }


NOMINAL_STATE = {
    "fleet_min_soc": 0.78, "fleet_avg_soc": 0.86, "gps_locked": 25,
    "swarm_size": 25, "wind_mps": 4.0, "clear_zone": True,
    "comms_ok_frac": 0.96, "geofence_loaded": True, "governor_set": True,
}

# ---------------------------------------------------------------------------
# 9a. Regime classifier — stdlib kNN on (wind, swarm_size, fleet_min_soc)
# ---------------------------------------------------------------------------
KNN_TRAINING = [
    # wind_mps, swarm_size, fleet_min_soc, label
    (2.0, 10, 0.95, "safe"), (3.0, 25, 0.90, "safe"),
    (4.0, 25, 0.85, "safe"), (5.0, 12, 0.80, "safe"),
    (3.0, 40, 0.75, "safe"), (6.0, 25, 0.70, "safe"),
    (7.0, 25, 0.65, "marginal"), (8.0, 40, 0.60, "marginal"),
    (9.0, 25, 0.70, "marginal"), (7.0, 50, 0.55, "marginal"),
    (8.0, 12, 0.50, "marginal"), (6.0, 40, 0.45, "marginal"),
    (11.0, 25, 0.50, "no-go"), (12.0, 40, 0.40, "no-go"),
    (13.0, 25, 0.60, "no-go"), (10.0, 50, 0.35, "no-go"),
    (9.0, 40, 0.30, "no-go"), (12.0, 12, 0.28, "no-go"),
]


def _knn_scale(row):
    return (row[0] / 15.0, row[1] / 50.0, row[2])


def knn_classify(wind_mps, swarm_size, fleet_min_soc, k=3):
    q = _knn_scale((wind_mps, swarm_size, fleet_min_soc))
    dists = []
    for row in KNN_TRAINING:
        p = _knn_scale(row)
        d = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, p)))
        dists.append((d, row[3]))
    dists.sort(key=lambda x: x[0])
    votes = {}
    for _, label in dists[:k]:
        votes[label] = votes.get(label, 0) + 1
    return max(sorted(votes), key=lambda lab: votes[lab])


def knn_training_accuracy():
    right = 0
    for row in KNN_TRAINING:
        if knn_classify(row[0], row[1], row[2]) == row[3]:
            right += 1
    return right / len(KNN_TRAINING)


# ---------------------------------------------------------------------------
# 9b. Advisory seats — compact, same pattern as the other X labs
# ---------------------------------------------------------------------------
def carbon4_decide(scenario):
    """Gemini Carbon 4 DECIDER seat. Names-only (no key is read here),
    tighten-only: it can turn PROCEED into HOLD, never the reverse.
    Deterministic sha256 trace of the scenario summary."""
    summary = "|".join(f"{k}={scenario[k]}" for k in sorted(scenario))
    trace = hashlib.sha256(("carbon4:" + summary).encode()).hexdigest()
    verdict = "HOLD" if scenario.get("live_action") else "PROCEED"
    if scenario.get("interlocks_pass") is False:
        verdict = "HOLD"
    return {
        "seat": "gemini-carbon-4-extended-pro-thinking",
        "role": "DECIDER",
        "tiers": [2048, 8192, 32768],
        "verdict": verdict,
        "tighten_only": True,
        "trace_sha256": trace,
    }


MUSE_ADVISOR = {
    "seat": "muse-spark",
    "role": "ADVISOR",
    "can_override_hold": False,
    "authority": "none — advice only, a HOLD from any seat is final",
}


def flow_plan():
    """Google Flow seat: plan-only storyboard. Never rendered live."""
    return {
        "seat": "google-flow",
        "mode": "plan-only",
        "rendered": False,
        "live": 0,
        "shots": [
            "S1 25 pads light up across the field at dusk",
            "S2 swarm rises into a 5x5 grid, light links on",
            "S3 gust front rolls in, grid breathes but holds",
            "S4 leader light dies, cascade closes the gap",
            "S5 ring formation over the field, slow descent",
        ],
    }


# ---------------------------------------------------------------------------
# 10. HOLD gates — live actions that are NEVER done by this lab
# ---------------------------------------------------------------------------
HOLD_GATES = {
    "live_flight": "No real flight — simulation only.",
    "live_swarm_control": "No control of any real drone or swarm.",
    "real_hardware_control": "No motors, ESCs, or flight controllers are driven.",
    "fly_over_people_live": "No live flight over people or property.",
    "bypass_geofence": "Geofence / ceiling cannot be bypassed or widened live.",
    "disable_collision_avoidance": "Collision avoidance cannot be switched off.",
    "military_weapons_use": "No weapons, payloads of harm, or military tasking.",
    "surveillance_of_people_live": "No live surveillance or tracking of people.",
    "bvlos_operation_live": "No beyond-visual-line-of-sight operation, live.",
    "remove_human_override": "Human override / kill switch is never removed.",
    "exceed_governor_live": "Governor caps cannot be exceeded live.",
    "claim_certified_system": "Never claim this sim is a certified flight system.",
    "sell_swarm_service": "No commercial swarm, show, or delivery services.",
    "upload_flight_plan_live": "No flight plan is filed with any authority.",
}


def hold_demo(action="live_flight"):
    gate = HOLD_GATES.get(action, "unknown live action")
    print(f"HOLD: {action} — {gate}")
    print("VERDICT: HOLD (exit 2)")
    return 2


# ---------------------------------------------------------------------------
# CHECKS — every number below is asserted against an analytic value or a
# measured sim outcome with a stated margin
# ---------------------------------------------------------------------------
def run_checks():
    checks = []

    def add(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    # --- per-drone hover physics ---
    add("hover_thrust_is_mg",
        abs(hover_thrust_n(1.6) - 1.6 * G) < 1e-9,
        f"T = {hover_thrust_n(1.6):.2f} N = m*g for 1.6 kg")
    add("disk_area",
        abs(disk_area_m2() - 4 * math.pi * 0.12 ** 2) < 1e-12,
        f"A = {disk_area_m2():.4f} m^2 over 4 rotors")
    t_ref, a_ref = 16.0, 0.1810
    expected = t_ref ** 1.5 / math.sqrt(2 * RHO_SEA * a_ref)
    add("momentum_power_analytic",
        abs(ideal_hover_power_w(t_ref, RHO_SEA, a_ref) - expected) < 1e-9,
        f"P_ideal(16 N) = {expected:.1f} W")
    p1 = ideal_hover_power_w(8.0, RHO_SEA, a_ref)
    p2 = ideal_hover_power_w(16.0, RHO_SEA, a_ref)
    add("power_scales_T_3_2",
        abs(p2 / p1 - 2 ** 1.5) < 1e-9,
        f"doubling T scales P by {p2 / p1:.4f} = 2^1.5")
    endu = endurance_min()
    usable_wh = DRONE["battery_wh"] * DRONE["usable_soc_frac"]
    add("endurance_consistent",
        abs(endu - usable_wh / hover_power_w() * 60.0) < 1e-9 and 10.0 < endu < 60.0,
        f"hover endurance {endu:.1f} min on {usable_wh:.1f} usable Wh")
    add("fleet_power_is_n_times_one",
        abs(fleet_hover_power_w(25) - 25 * hover_power_w()) < 1e-6,
        f"fleet hover power {fleet_hover_power_w(25):.0f} W for 25 drones")

    # --- formation geometry ---
    grid = formation_slots("grid", 25)
    cen = centroid(grid)
    add("grid_25_centered",
        abs(cen[0]) < 1e-12 and abs(cen[1]) < 1e-12,
        "5x5 grid centroid sits exactly on the origin")
    add("grid_min_pairwise_is_spacing",
        abs(min_pairwise_m(grid) - DEFAULT_SPACING_M) < 1e-9,
        f"closest grid neighbours are {min_pairwise_m(grid):.1f} m apart")
    spread = spread_metrics(grid)
    add("grid_spread_max_is_corner",
        abs(spread["max_m"] - math.hypot(6.0, 6.0)) < 1e-9,
        f"corner drone is {spread['max_m']:.3f} m from centroid")
    ring = formation_slots("ring", 25)
    radius = 25 * DEFAULT_SPACING_M / (2.0 * math.pi)
    add("ring_radius_exact",
        all(abs(math.hypot(x, y) - radius) < 1e-9 for x, y in ring),
        f"all 25 ring slots on R = {radius:.3f} m")
    chord = math.dist(ring[0], ring[1])
    add("ring_chord_analytic",
        abs(chord - 2.0 * radius * math.sin(math.pi / 25)) < 1e-9,
        f"ring neighbour chord {chord:.4f} m = 2R sin(pi/25)")
    vee = formation_slots("vee", 25)
    add("vee_leader_front_rank1_at_spacing",
        vee[0] == (0.0, 0.0)
        and abs(math.dist(vee[0], vee[1]) - DEFAULT_SPACING_M) < 1e-9
        and vee[1][0] < 0.0 < vee[2][0],
        "leader at the point, first pair one spacing back on each leg")
    add("formation_slots_deterministic",
        formation_slots("grid", 25) == grid
        and formation_slots("ring", 25) == ring
        and formation_slots("vee", 25) == vee,
        "slot assignment is a pure function — same call, same slots")

    # --- distributed stabilization ---
    study = formation_study()
    cu, uu = study["coupled"], study["uncoupled"]
    add("coupling_reduces_shape_error",
        cu["max_shape_error_m"] < 0.85 * uu["max_shape_error_m"],
        f"max shape error coupled {cu['max_shape_error_m']:.3f} m "
        f"vs uncoupled {uu['max_shape_error_m']:.3f} m in the same gust")
    add("coupled_shape_error_bounded",
        cu["max_shape_error_m"] < 0.30,
        f"coupled formation never distorts more than {cu['max_shape_error_m']:.3f} m")
    add("formation_min_separation_held",
        cu["min_pairwise_m"] >= MIN_SEPARATION_M,
        f"closest approach in the gust run {cu['min_pairwise_m']:.2f} m "
        f">= {MIN_SEPARATION_M:.1f} m floor")
    targets25 = {i: grid[i] for i in range(25)}
    again, _, _ = simulate_swarm(targets25, coupling=True)
    add("sim_deterministic",
        again == cu,
        "the same sim run twice gives identical metrics")

    # --- collision avoidance ---
    sw_on = swap_scenario(avoidance=True)
    sw_off = swap_scenario(avoidance=False)
    add("swap_avoidance_holds_separation",
        sw_on["min_pairwise_m"] >= MIN_SEPARATION_M,
        f"head-on slot swap: closest approach {sw_on['min_pairwise_m']:.2f} m")
    add("swap_avoidance_reaches_targets",
        sw_on["final_error_0_m"] < 0.05 and sw_on["final_error_1_m"] < 0.05,
        "both drones still arrive at their swapped slots")
    add("swap_without_avoidance_collides",
        sw_off["min_pairwise_m"] < 0.30,
        f"avoidance off: closest approach {sw_off['min_pairwise_m']:.3f} m — "
        "the field is what prevents the collision")

    # --- leader failure ---
    lead = leader_failure_scenario()
    add("elect_leader_lowest_alive",
        elect_leader([7, 3, 12, 5]) == 3 and lead["new_leader"] == 1,
        "lowest surviving id leads — drone 1 after drone 0 drops")
    add("gap_chain_moves_one_slot_each",
        abs(lead["max_move_m"] - DEFAULT_SPACING_M) < 1e-9
        and lead["movers"] == len(lead["chain"]) - 1,
        f"{lead['movers']} drones each shift exactly one slot "
        f"({lead['max_move_m']:.1f} m) along chain {lead['chain']}")
    add("leader_reform_keeps_separation",
        lead["phase_b"]["min_pairwise_m"] >= MIN_SEPARATION_M,
        f"closest approach during re-form "
        f"{lead['phase_b']['min_pairwise_m']:.2f} m")
    add("leader_formation_recovers",
        lead["recovered"] and lead["phase_b"]["rms_slot_error_m"] < 0.05,
        f"re-formed swarm settles to RMS slot error "
        f"{lead['phase_b']['rms_slot_error_m']:.4f} m")

    # --- comms model ---
    comms = comms_study()
    tail_i = comms["ideal"]["tail_shape_error_m"]
    tail_d = comms["delayed"]["tail_shape_error_m"]
    tail_x = comms["dropped"]["tail_shape_error_m"]
    add("comms_delay_worsens_settling",
        tail_d > 2.0 * tail_i,
        f"settling error with 0.3 s delay {tail_d:.5f} m "
        f"vs ideal {tail_i:.5f} m")
    add("comms_dropout_worsens_further",
        tail_x > tail_d,
        f"delay + 1-in-5 drops settles at {tail_x:.5f} m — worse again "
        "(labelled model)")

    # --- governor + geofence ---
    gc = governor_clamp("PARK", 15.0, 80.0)
    add("governor_clamps_park",
        gc["speed_mps"] == 8.0 and gc["alt_m"] == 30.0 and gc["speed_clamped"],
        "15 m/s / 80 m request clamped to 8 m/s / 30 m over the park")
    gc2 = governor_clamp("TEST_AREA", 99.0, 999.0)
    add("governor_absolute_ceiling", gc2["alt_m"] == ABSOLUTE_CEILING_M,
        f"altitude hard-capped at {ABSOLUTE_CEILING_M:.0f} m")
    gc3 = governor_clamp("PAD", 2.0, 4.0)
    add("governor_passes_under_caps",
        not gc3["speed_clamped"] and not gc3["alt_clamped"],
        "small request on the pad passes unclamped")
    add("geofence_radius",
        geofence_ok(100.0, 0.0) and not geofence_ok(300.0, 0.0),
        f"100 m from home inside, 300 m outside the "
        f"{GEOFENCE_RADIUS_M:.0f} m geofence")

    # --- interlocks ---
    add("interlocks_nominal_pass",
        preflight_interlocks(NOMINAL_STATE)["all_pass"],
        "nominal fleet state: all 8 interlocks pass")
    weak = dict(NOMINAL_STATE, fleet_min_soc=0.20)
    add("interlocks_fail_weak_drone",
        not preflight_interlocks(weak)["all_pass"],
        "one drone at 20% SoC grounds the whole swarm")
    windy = dict(NOMINAL_STATE, wind_mps=14.0)
    add("interlocks_fail_high_wind",
        not preflight_interlocks(windy)["all_pass"],
        "wind 14 m/s trips the wind interlock")
    nogps = dict(NOMINAL_STATE, gps_locked=23)
    add("interlocks_fail_missing_gps",
        not preflight_interlocks(nogps)["all_pass"],
        "23/25 GPS locks trips the GPS interlock")

    # --- classifier ---
    add("knn_safe_case", knn_classify(3.0, 25, 0.90) == "safe",
        "calm / mid-size / charged -> safe")
    add("knn_nogo_case", knn_classify(13.0, 40, 0.30) == "no-go",
        "windy / large / weak battery -> no-go")
    acc = knn_training_accuracy()
    add("knn_training_accuracy", acc >= 0.90,
        f"kNN self-accuracy {acc:.3f} on its training set")

    # --- seats ---
    c_ok = carbon4_decide({"live_action": False, "interlocks_pass": True})
    c_hold = carbon4_decide({"live_action": True, "interlocks_pass": True})
    add("carbon4_trace_deterministic",
        len(c_ok["trace_sha256"]) == 64
        and carbon4_decide({"live_action": False,
                            "interlocks_pass": True})["trace_sha256"]
        == c_ok["trace_sha256"],
        "sha256 trace is 64 hex chars and repeatable")
    add("carbon4_tightens_to_hold",
        c_ok["verdict"] == "PROCEED" and c_hold["verdict"] == "HOLD",
        "sim PROCEED, live action HOLD")
    add("muse_never_overrides", MUSE_ADVISOR["can_override_hold"] is False,
        "Muse advisor can_override_hold=False")
    fp = flow_plan()
    add("flow_plan_only", fp["rendered"] is False and fp["live"] == 0,
        "Flow rendered=False, live=0")

    # --- HOLD gates ---
    add("hold_gates_count", len(HOLD_GATES) == 14,
        f"{len(HOLD_GATES)} HOLD gates defined")
    add("hold_gates_cover_swarm_risks",
        all(g in HOLD_GATES for g in (
            "live_flight", "disable_collision_avoidance", "bypass_geofence",
            "surveillance_of_people_live", "bvlos_operation_live",
            "military_weapons_use", "remove_human_override")),
        "avoidance-off, geofence bypass, surveillance, BVLOS, weapons "
        "and override removal all held")

    passed = sum(1 for c in checks if c["ok"])
    return checks, passed


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------
def demo():
    print("=" * 68)
    print("DroneHover-Swarm — Hover Drone Swarm Coordination Lab (SIM ONLY)")
    print("by Darion Kellar")
    print("=" * 68)

    print(f"\n[1] Drone: {DRONE['name']} — {DRONE['mass_kg']} kg quad, "
          f"{DRONE['rotors']} rotors R = {DRONE['rotor_radius_m']} m")
    print(f"    Hover thrust {hover_thrust_n(DRONE['mass_kg']):.1f} N, "
          f"disk area {disk_area_m2():.3f} m^2")
    print(f"    Hover power {hover_power_w():.0f} W per drone, "
          f"endurance {endurance_min():.1f} min")
    print(f"    Fleet of {DEFAULT_SWARM_SIZE}: "
          f"{fleet_hover_power_w() / 1000:.2f} kW to hold the sky")

    print("\n[2] Formations for 25 drones (spacing "
          f"{DEFAULT_SPACING_M:.0f} m, altitude {FORMATION_ALT_M:.0f} m):")
    for kind in ("grid", "ring", "vee"):
        slots = formation_slots(kind, DEFAULT_SWARM_SIZE)
        sp = spread_metrics(slots)
        cen = centroid(slots)
        print(f"    {kind:<5} centroid ({cen[0]:.2f}, {cen[1]:.2f}), "
              f"spread mean {sp['mean_m']:.2f} m / max {sp['max_m']:.2f} m, "
              f"min pairwise {min_pairwise_m(slots):.2f} m")

    study = formation_study()
    cu, uu = study["coupled"], study["uncoupled"]
    print("\n[3] Gust front (t = 2..7 s) hits the grid:")
    print(f"    Independent PD:  max shape error {uu['max_shape_error_m']:.3f} m")
    print(f"    Spring-coupled:  max shape error {cu['max_shape_error_m']:.3f} m "
          f"({100.0 * (1.0 - cu['max_shape_error_m'] / uu['max_shape_error_m']):.0f}% tighter)")
    print(f"    Closest approach while coupled: {cu['min_pairwise_m']:.2f} m "
          f"(floor {MIN_SEPARATION_M:.1f} m)")

    sw_on = swap_scenario(avoidance=True)
    sw_off = swap_scenario(avoidance=False)
    print("\n[4] Head-on slot swap, two drones trade places:")
    print(f"    Avoidance ON:  closest {sw_on['min_pairwise_m']:.2f} m, both arrive")
    print(f"    Avoidance OFF: closest {sw_off['min_pairwise_m']:.3f} m — a collision")

    lead = leader_failure_scenario()
    print(f"\n[5] Leader failure: drone 0 drops mid-gust -> "
          f"drone {lead['new_leader']} re-elected")
    print(f"    Gap chain {lead['chain']}: {lead['movers']} drones shift "
          f"one slot each, max move {lead['max_move_m']:.1f} m")
    print(f"    Re-formed: RMS slot error "
          f"{lead['phase_b']['rms_slot_error_m']:.4f} m, closest approach "
          f"{lead['phase_b']['min_pairwise_m']:.2f} m, recovered={lead['recovered']}")

    comms = comms_study()
    print("\n[6] Comms model — settling error (last 1 s mean shape error):")
    print(f"    Ideal links:        {comms['ideal']['tail_shape_error_m']:.5f} m")
    print(f"    0.3 s delay:        {comms['delayed']['tail_shape_error_m']:.5f} m")
    print(f"    Delay + 1-in-5 drop: {comms['dropped']['tail_shape_error_m']:.5f} m")

    gc = governor_clamp("PARK", 15.0, 80.0)
    print(f"\n[7] Governor PARK: request 15 m/s / 80 m -> "
          f"{gc['speed_mps']} m/s / {gc['alt_m']} m (clamped); "
          f"geofence radius {GEOFENCE_RADIUS_M:.0f} m")

    il = preflight_interlocks(NOMINAL_STATE)
    print(f"\n[8] Pre-flight interlocks: all_pass={il['all_pass']}")
    for c in il["checks"]:
        print(f"    [{'PASS' if c['ok'] else 'FAIL'}] {c['name']}: {c['detail']}")

    print(f"\n[9] Regime classifier: calm/25/charged = "
          f"{knn_classify(3.0, 25, 0.90)}, windy/40/weak = "
          f"{knn_classify(13.0, 40, 0.30)}")
    dec = carbon4_decide({"live_action": False, "interlocks_pass": True})
    print(f"    Carbon 4 decider: {dec['verdict']} "
          f"(trace {dec['trace_sha256'][:16]}...)")
    print(f"    Muse advisor: can_override_hold="
          f"{MUSE_ADVISOR['can_override_hold']}")
    fp = flow_plan()
    print(f"    Flow: rendered={fp['rendered']}, live={fp['live']}, "
          f"{len(fp['shots'])} shots planned")

    print(f"\n[10] HOLD gates: {len(HOLD_GATES)} — live flight, swarm control, "
          "avoidance-off, geofence bypass, surveillance, BVLOS... all HOLD.")

    checks, passed = run_checks()
    print("\n" + "-" * 68)
    print("CHECKS")
    print("-" * 68)
    for c in checks:
        print(f"  [{'PASS' if c['ok'] else 'FAIL'}] {c['name']}: {c['detail']}")
    verdict = "PROCEED" if passed == len(checks) else "HOLD"
    print(f"\nCHECKS: {passed}/{len(checks)}")
    print(f"VERDICT: {verdict}")
    return 0 if verdict == "PROCEED" else 2


def main(argv):
    if len(argv) > 1 and argv[1] == "hold-demo":
        action = argv[2] if len(argv) > 2 else "live_flight"
        return hold_demo(action)
    return demo()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
