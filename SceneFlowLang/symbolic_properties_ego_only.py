import SG_Primitives as P
from Property import Property, SubpropertyWrapper
from functools import partial
import math

from SymbolicEntity import SymbolicEntity
from SymbolicProperty import SymbolicProperty

VEHICLE_CLASSES = ['car', 'truck', 'van', 'bus', 'motorcycle', 'bicycle', 'vehicle']
MOTOR_VEHICLE_CLASSES = ['car', 'truck', 'van', 'bus', 'motorcycle', 'vehicle']
ROAD_USER_CLASSES = ['car', 'truck', 'van', 'bus', 'motorcycle', 'bicycle', 'vehicle', 'person']

STOPPED_SPEED = 0.2  # this is a bit fast, but it's what CARLA defines internally
# STOPPED_SPEED = 1e-4


def entity_lanes(entity):
    lanes = partial(P.relSet, entity, "isIn")
    return lanes


def entity_roads(entity):
    roads = partial(P.relSet, entity_lanes(entity), "isIn")
    return roads


def entity_junctions(entity):
    junctions = partial(P.relSet, entity_roads(entity), "isIn")
    return junctions


def stop_signs_for(entity):
    entity_lanes = partial(P.relSet, entity, "isIn")
    entity_roads = partial(P.relSet, entity_lanes, "isIn")
    entity_junctions = partial(P.relSet, entity_roads, "isIn")
    entities_in_junction_with = partial(P.relSet, partial(P.relSet, partial(P.relSet, entity_junctions, "isIn",
                                                                                edge_type='incoming'), "isIn",
                                                              edge_type='incoming'), "isIn", edge_type='incoming')
    entity_if_in_junction = partial(P.intersection, entities_in_junction_with, entity)
    entity_if_not_in_junction = partial(P.symmetric_difference, entity_if_in_junction, entity)
    potential_stop_signs_naive = partial(P.intersection,
                                                 partial(P.relSet, partial(P.relSet, entity_if_not_in_junction, "isIn"),
                                                         "controlsTrafficOf", edge_type="incoming"),
                                                 partial(P.filterByAttr, "G", "name", "stop_sign*"))
    potential_stop_signs = partial(P.union,
                                           potential_stop_signs_naive,
                                           partial(P.intersection,
                                                   partial(P.relSet,
                                                           partial(P.intersection,
                                                                   partial(P.relSet,
                                                                           partial(P.relSet, entity_if_not_in_junction,
                                                                                   "isIn"), "toLeftOf"),
                                                                   partial(P.relSet,
                                                                           partial(P.relSet, entity_if_not_in_junction,
                                                                                   "isIn"), "laneChange")
                                                                   ), "controlsTrafficOf", edge_type="incoming"),
                                                   partial(P.filterByAttr, "G", "name", "stop_sign*"))
                                           )
    entity_opp_lanes = partial(P.relSet, entity_lanes, "opposes")
    stop_signs_for_opp_lanes = partial(P.intersection,
                                           partial(P.relSet, entity_opp_lanes, "controlsTrafficOf",
                                                   edge_type="incoming"),
                                           partial(P.filterByAttr, "G", "name", "stop_sign*"))
    stop_signs = partial(P.difference, potential_stop_signs, stop_signs_for_opp_lanes)
    return stop_signs


def has_stop_signs(entity):
    return partial(P.gt, partial(P.size, stop_signs_for(entity)), 0)


def set_size_eq(nodes, size):
    return partial(P.eq, partial(P.size, nodes), size)

def sym_entities_not_equal(entity1, entity2):
    # two symbolic entities are not equal if they each are sets of size 1 and the set1 - set2 is size 1
    return partial(P.logic_and,
                   partial(P.logic_and,set_size_eq(entity1, 1), set_size_eq(entity2, 1)),
                         set_size_eq(partial(P.difference, entity1, entity2), 1)
                   )

def non_empty(nodes):
    return partial(P.gt, partial(P.size, nodes), 0)


EGO = partial(P.filterByAttr, "G", "name", "ego")

YIELD_JUNCTION = SymbolicEntity('yield_junction', base_filter=['junction'])
YIELD_JUNCTION_ROADS = partial(P.relSet, YIELD_JUNCTION, "isIn", edge_type="incoming")

YIELD_VEHICLE1 = SymbolicEntity('yield_vehicle_1', VEHICLE_CLASSES)
YIELD_VEHICLE2 = SymbolicEntity('yield_vehicle_2', VEHICLE_CLASSES)
ROAD_USER1 = SymbolicEntity('road_user_1', ROAD_USER_CLASSES)

RIGHT_LANE = SymbolicEntity('Right Lane', ['lane'])
LEFT_LANE = SymbolicEntity('Left Lane', ['lane'])
MIDDLE_LANE = SymbolicEntity('Middle Lane', ['lane'])

def is_in_junction(vehicle, junction):
    return partial(P.eq,partial(P.size, partial(P.intersection, entity_junctions(vehicle), junction)), 1)

EGO_ONLY_IN_JUNCTION = partial(P.eq, partial(P.size, partial(P.difference, entity_roads(EGO), YIELD_JUNCTION_ROADS)), 0)

symbolic_stop_sign_yield = SymbolicProperty(
    "821_vehicle2_needs_to_yield_to_vehicle1_stop",
    "((((v1_at_junc) & !(v2_at_junc) & v2_has_stop) & X((v1_at_junc & v2_at_junc))) -> X(X(((v2_at_junc & !v2_only_in_junc) U !(v1_at_junc)))))",
    [("v1_at_junc", is_in_junction(YIELD_VEHICLE1, YIELD_JUNCTION)),
     ("v2_at_junc", is_in_junction(EGO, YIELD_JUNCTION)),
     ("v2_only_in_junc", EGO_ONLY_IN_JUNCTION),
     ("v2_has_stop", has_stop_signs(EGO))],
    [YIELD_VEHICLE1, YIELD_JUNCTION])

symbolic_stop_sign_tie_left_yield = SymbolicProperty(
    "820_vehicle2_needs_to_yield_to_vehicle1_stop_tie",
    "((((!v1_at_junc) & !(v2_at_junc) & v2_has_stop & v1_has_stop) & X((v1_at_junc & v2_at_junc & v2_right_of_v1))) -> X(X(((v2_at_junc & !v2_only_in_junc) U !(v1_at_junc)))))",
    [("v1_at_junc", is_in_junction(YIELD_VEHICLE1, YIELD_JUNCTION)),
     ("v2_at_junc", is_in_junction(EGO, YIELD_JUNCTION)),
     ("v2_only_in_junc", EGO_ONLY_IN_JUNCTION),
     ("v1_has_stop", has_stop_signs(YIELD_VEHICLE1)),
     ("v2_has_stop", has_stop_signs(EGO)),
     ("v2_right_of_v1", partial(P.eq,
                                partial(P.size,
                                        partial(P.intersection,
                                                partial(P.relSet, EGO, "toRightOf"),
                                                YIELD_VEHICLE1)),
                                1))
     ],
    [YIELD_VEHICLE1, YIELD_JUNCTION])


def is_emergency_vehicle(vehicle):
    return partial(P.eq,
            partial(P.size,
                    partial(P.intersection,
                            vehicle,
                            partial(P.filterByAttr, vehicle, "carla_type_id", lambda x: x is not None and 'ambulance' in x)
                            )
                    ),
            1)


def has_lights_on(vehicle, light_type):
    return partial(P.eq,
            partial(P.size,
                    partial(P.intersection,
                            vehicle,
                            partial(P.filterByAttr, vehicle, light_type, lambda x: x)
                            )
                    ),
            1)


def has_emergency_lights_on(vehicle):
    return partial(P.logic_or, has_lights_on(vehicle, 'light_Special1'), has_lights_on(vehicle, 'light_Special2'))


symbolic_stop_sign_yield_to_emergency = SymbolicProperty(
    "829_vehicle2_needs_to_yield_to_emergency",
    "(((!(v2_at_junc)) & X((emergency_at_junc & emergency_has_lights & v2_at_junc & v1_not_v2))) -> X(X(((v2_at_junc & !v2_only_in_junc) U !(emergency_at_junc)))))",
    [("emergency_at_junc", partial(P.logic_and,
                                   is_in_junction(YIELD_VEHICLE1, YIELD_JUNCTION),
                                          is_emergency_vehicle(YIELD_VEHICLE1))
      ),
     ('emergency_has_lights', has_emergency_lights_on(YIELD_VEHICLE1)),
     ("v2_at_junc", is_in_junction(EGO, YIELD_JUNCTION)),
     ("v2_only_in_junc", EGO_ONLY_IN_JUNCTION),
     ("v2_has_stop", has_stop_signs(EGO)),
      ("v1_not_v2", sym_entities_not_equal(YIELD_VEHICLE1, EGO))],
    [YIELD_VEHICLE1, YIELD_JUNCTION])


def is_too_close(vehicle2, vehicle1, distance):
    """Return a partial that calculates if vehicle2 is too close to vehicle 1"""
    close_to_vehicle_2 = entities_within(vehicle2, distance)
    # if the intersection of the things "close to" vehicle2 with vehicle 1 is size 1, then vehicle 2 was near vehicle 1
    return set_size_eq(partial(P.intersection, close_to_vehicle_2, vehicle1), 1)


def debounced_is_too_close(vehicle2, vehicle1, distance, hold_frames=1):
    """
    Like is_too_close but tolerates up to `hold_frames` consecutive False frames
    (false dropouts) before actually going False.  State is keyed by
    (distance, vehicle_name) so different distance tiers and different
    track_car assignments are fully independent.

    hold_frames=1  → a single-frame dropout is ignored (output stays True).
    hold_frames=0  → identical to plain is_too_close (no hold).
    """
    close_to_v2 = entities_within(vehicle2, distance)
    intersection_set = partial(P.intersection, close_to_v2, vehicle1)

    hold_state = {}    # {(distance, v1_name): remaining hold frames}
    output_state = {}  # {(distance, v1_name): current debounced boolean}

    def _held(v2_nodes, v1_nodes, intersection):
        raw = isinstance(intersection, set) and len(intersection) == 1

        # Best-effort key: use the vehicle's node name when available,
        # fall back to a sentinel so hold logic still applies for brief
        # dropouts where the node is temporarily absent from the graph.
        node = next(iter(v1_nodes), None) if isinstance(v1_nodes, set) else None
        key = (distance, node.name if node is not None else "_unresolved")

        if raw:
            hold_state[key] = hold_frames
            output_state[key] = True
        else:
            remaining = hold_state.get(key, 0)
            if remaining > 0:
                hold_state[key] = remaining - 1
                output_state[key] = True
            else:
                output_state[key] = False

        return output_state[key]

    return partial(_held, vehicle2, vehicle1, intersection_set)


def debounced_within_headway(vehicle2, vehicle1, max_headway_s, hold_frames=1,
                              min_ego_speed=0.1, heading_threshold_rad=math.pi / 4):
    """
    Like debounced_is_too_close but gates on headway (time-gap) instead of distance.

        headway = longitudinal_distance_to_vehicle / ego_speed  [seconds]

    True when the lead vehicle is ahead of ego AND headway <= max_headway_s.
    When ego speed is at or below min_ego_speed the predicate is always False
    (headway is undefined / infinite at standstill).

    Tolerates up to `hold_frames` consecutive False frames before going False
    (mirrors the dropout-hold behaviour of debounced_is_too_close).

    vehicle2 is expected to be the ego entity; vehicle1 is the tracked vehicle.
    """
    hold_state   = {}   # {key: remaining hold frames}
    output_state = {}   # {key: current debounced boolean}

    def _held(ego_nodes, v1_nodes):
        ex   = P.getAttr(ego_nodes, "position_x")
        ey   = P.getAttr(ego_nodes, "position_y")
        eyaw = P.getAttr(ego_nodes, "yaw")
        espd = P.getAttr(ego_nodes, "velocity")
        vx   = P.getAttr(v1_nodes,  "position_x")
        vy   = P.getAttr(v1_nodes,  "position_y")

        node = next(iter(v1_nodes), None) if isinstance(v1_nodes, set) else None
        key  = (max_headway_s, node.name if node is not None else "_unresolved")

        if any(v is None for v in [ex, ey, eyaw, espd, vx, vy]) or espd <= min_ego_speed:
            raw = False
        else:
            hx  = math.cos(eyaw)
            hy  = math.sin(eyaw)
            lon = hx * (vx - ex) + hy * (vy - ey)
            raw = lon > 0 and (lon / espd) <= max_headway_s

        if raw:
            hold_state[key]   = hold_frames
            output_state[key] = True
        else:
            remaining = hold_state.get(key, 0)
            if remaining > 0:
                hold_state[key]   = remaining - 1
                output_state[key] = True
            else:
                output_state[key] = False

        return output_state[key]

    return partial(_held, vehicle2, vehicle1)


def is_too_close_bike(vehicle2, vehicle1, add_buffer=False):
    """Return a partial that calculates if vehicle2 is too close to vehicle 1"""
    close_to_vehicle_2 = partial(P.relSet, vehicle2, "safe_hazard")
    if add_buffer:
        close_to_vehicle_2 = partial(P.union, close_to_vehicle_2, partial(P.relSet, vehicle2, "near_coll"))
    # if the intersection of the things "close to" vehicle2 with vehicle 1 is size 1, then vehicle 2 was near vehicle 1
    return set_size_eq(partial(P.intersection, close_to_vehicle_2, vehicle1), 1)


def is_too_close_emergency(vehicle2, vehicle1):
    """Return a partial that calculates if vehicle2 is too close to vehicle 1"""
    close_to_vehicle_2 = partial(P.union, partial(P.relSet, vehicle2, "safe_hazard"),
                                 partial(P.relSet, vehicle2, "near_coll"))
    close_to_vehicle_2 = partial(P.union, close_to_vehicle_2,
                                 partial(P.relSet, vehicle2, "super_near"))
    close_to_vehicle_2 = partial(P.union, close_to_vehicle_2,
                                 partial(P.relSet, vehicle2, "very_near"))
    close_to_vehicle_2 = partial(P.union, close_to_vehicle_2,
                                 partial(P.relSet, vehicle2, "near"))
    close_to_vehicle_2 = partial(P.union, close_to_vehicle_2,
                                 partial(P.relSet, vehicle2, "visible"))
    # if the intersection of the things "close to" vehicle2 with vehicle 1 is size 1, then vehicle 2 was near vehicle 1
    return set_size_eq(partial(P.intersection, close_to_vehicle_2, vehicle1), 1)

def filterByGraphAttr(key, condition, sg, entity_mapping):
    """Like filterByAttr but reads from sg.graph metadata instead of nodes."""
    val = sg.graph.get(key, None)
    if callable(condition):
        return condition(val)
    return val == condition

def same_lane(vehicle1, vehicle2):
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), entity_lanes(vehicle2)))

def behind(vehicle1, vehicle2):
    """Return a partial that calcs if vehicle2 is behind vehicle 1"""
    behind_v1_entities = partial(P.union, partial(P.relSet, vehicle1, "atDRearOf"),
                                 partial(P.relSet, vehicle1, "atSRearOf"))
    is_behind_v1 = set_size_eq(partial(P.intersection, behind_v1_entities, vehicle2), 1)
    front_v2_entities = partial(P.union, partial(P.relSet, vehicle2, "inDFrontOf"),
                                 partial(P.relSet, vehicle2, "inSFrontOf"))
    is_front_v2 = set_size_eq(partial(P.intersection, front_v2_entities, vehicle1), 1)
    return partial(P.logic_and, is_behind_v1, is_front_v2)

def side_behind(vehicle1, vehicle2):
    """Return a partial that calcs if ego is diag behind vehicle 1 (stricter version of behind)"""
    behind_v1_entities = partial(P.relSet, vehicle1, "atSRearOf")
    is_behind_v1 = set_size_eq(partial(P.intersection, behind_v1_entities, vehicle2), 1)

    front_v2_entities = partial(P.relSet, vehicle2, "inSFrontOf")
    is_front_v2 = set_size_eq(partial(P.intersection, front_v2_entities, vehicle1), 1)

    return partial(P.logic_and, is_behind_v1, is_front_v2)

def direct_behind(vehicle1, vehicle2):
    """Return a partial that calcs if ego is behind vehicle 1 (stricter version of behind)"""
    behind_v1_entities = partial(P.relSet, vehicle1, "atDRearOf")
    is_behind_v1 = set_size_eq(partial(P.intersection, behind_v1_entities, vehicle2), 1)

    front_v2_entities = partial(P.relSet, vehicle2, "inDFrontOf")
    is_front_v2 = set_size_eq(partial(P.intersection, front_v2_entities, vehicle1), 1)
    return partial(P.logic_and, is_behind_v1, is_front_v2)

def behind_kinematic(vehicle1, vehicle2, longitudinal_threshold=0.0,
                     lateral_tolerance=None, heading_threshold_rad=math.pi / 4):
    """
    AP: vehicle1 is behind vehicle2 based on kinematic state.

    Guards against spurious matches by requiring both vehicles to have
    similar headings (i.e. travelling in roughly the same direction).
    heading_threshold_rad: max allowed yaw difference (default π/2 = 90°,
                           use π/4 = 45° for stricter same-direction requirement)
    """
    def _behind_check(v1_nodes, v2_nodes, long_thresh, lat_tol, head_thresh):
        x1    = P.getAttr(v1_nodes, "position_x")
        y1    = P.getAttr(v1_nodes, "position_y")
        x2    = P.getAttr(v2_nodes, "position_x")
        y2    = P.getAttr(v2_nodes, "position_y")
        yaw1  = P.getAttr(v1_nodes, "yaw")
        yaw2  = P.getAttr(v2_nodes, "yaw")

        if any(v is None for v in [x1, y1, x2, y2, yaw1, yaw2]):
            return False

        # ── heading similarity gate ─────────────────────────────────────────
        # Vehicles must be travelling in roughly the same direction,
        # otherwise "behind" is geometrically meaningless in a traffic sense
        yaw_diff = abs(yaw1 - yaw2) % (2 * math.pi)
        yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)
        if yaw_diff > head_thresh:
            return False

        # ── displacement vector: from v2 toward v1 ──────────────────────────
        dx = x1 - x2
        dy = y1 - y2

        # ── v2 heading unit vector ───────────────────────────────────────────
        hx = math.cos(yaw2)
        hy = math.sin(yaw2)

        # ── longitudinal projection: negative → v1 is behind v2 ────────────
        longitudinal = hx * dx + hy * dy
        behind_check = longitudinal < long_thresh

        # ── optional lateral alignment (same-lane) check ────────────────────
        if lat_tol is not None:
            lx, ly = -hy, hx
            lateral = abs(lx * dx + ly * dy)
            return behind_check and lateral <= lat_tol

        return behind_check

    return partial(_behind_check, vehicle1, vehicle2,
                   longitudinal_threshold, lateral_tolerance, heading_threshold_rad)

# toLeftOf, toRightOf, inSFrontOf, atSRearOf

# including only toLeftOf and toRight of. Doesn't include vehicles at an angle (front-side, rear-side)
def beside(vehicle1, vehicle2):
    """Return a partial that calcs if vehicle2 is behind vehicle 1"""
    side_v1_entities = partial(P.union, partial(P.relSet, vehicle1, "toLeftOf"),
                                 partial(P.relSet, vehicle1, "toRightOf"))
    is_side_v1 = set_size_eq(partial(P.intersection, side_v1_entities, vehicle2), 1) #is_side_v1 is true if vehicle 2 is at the side of vehicle 1
    side_v2_entities = partial(P.union, partial(P.relSet, vehicle2, "toLeftOf"),
                                 partial(P.relSet, vehicle2, "toRightOf"))
    is_side_v2 = set_size_eq(partial(P.intersection, side_v2_entities, vehicle1), 1)
    return partial(P.logic_and, is_side_v1, is_side_v2)

def is_moving_carla(entity):
    """Returns a partial evaluating if entity is moving"""
    return partial(P.eq, partial(P.size,
                          partial(P.filterByAttr, entity, "carla_speed",
                                  (lambda a: a is not None and P.gt(a, STOPPED_SPEED)))), 1)

"""Scenario Extraction:  My atomic propositions"""
def is_moving_video(entity):
    """Returns a partial evaluating if entity is moving"""
    return partial(P.eq, partial(P.size,
                          partial(P.filterByAttr, entity, "velocity", 
                                  (lambda a: a is not None and P.gt(a, 0.2)))), 1) # less than 0.2  is considered noise.

def lateral_velocity_leftward(ego, entity):
    """
    True iff entity is moving toward ego's lane at > 0.2 m/s.
    Projects entity's city-frame velocity onto ego's lateral axis, then checks
    directionality: right-side entities (lat < 0) must have positive lat_vel,
    left-side entities (lat > 0) must have negative lat_vel. This rejects
    vehicles on the far side of ego whose apparent lateral velocity is an
    artifact of ego rotating through a curve.
    """
    def _check(ego_nodes, v_nodes):
        ego_yaw = P.getAttr(ego_nodes, "yaw")
        ex      = P.getAttr(ego_nodes, "position_x")
        ey      = P.getAttr(ego_nodes, "position_y")
        px      = P.getAttr(v_nodes,   "position_x")
        py      = P.getAttr(v_nodes,   "position_y")
        vx      = P.getAttr(v_nodes,   "velocity_x")
        vy      = P.getAttr(v_nodes,   "velocity_y")
        if any(v is None for v in [ego_yaw, ex, ey, px, py, vx, vy]):
            return False
        lat     = -math.sin(ego_yaw) * (px - ex) + math.cos(ego_yaw) * (py - ey)
        lat_vel = -vx * math.sin(ego_yaw) + vy * math.cos(ego_yaw)
        return lat != 0 and (lat * lat_vel < 0) and abs(lat_vel) > 0.2
    return partial(_check, ego, entity)

def velocity_is_greater_than(entity, speed):
    """Returns a partial evaluating if entity is moving faster than speed"""
    return partial(P.eq, partial(P.size,
                          partial(P.filterByAttr, entity, "velocity",
                                  (lambda a: a is not None and P.gt(a, speed)))), 1)

# def same_heading(vehicle1, vehicle2):
#     """returns a partial evaluating if two vehicles have the same heading (with small error)"""
#     partial(P.filterByAttr, vehicle1, "yaw")

def nearest_same_lane(vehicle1, vehicle2, switch_frames=1, hold_frames=0):
    """
    True iff vehicle1 is in the same lane as vehicle2 AND is the debounced
    nearest same-lane vehicle to ego by Euclidean distance.

    Tier buckets gate candidates; Euclidean distance breaks ties within a tier.
    An incumbent/challenger state machine suppresses transient flickering:
    a new raw_best must appear for `switch_frames` consecutive frames before
    displacing the current incumbent, which is then frozen for `hold_frames`
    additional frames before it can be challenged again.

    Default switch_frames=1, hold_frames=0 gives the original behaviour
    (switch immediately, no hold) for all existing callers.
    """

    # All vehicles in vehicle2's (ego's) lane, via incoming isIn edges
    ego_lane_vehicles = partial(P.relSet, entity_lanes(vehicle2), "isIn", edge_type="incoming")

    # Cumulative distance sets centered on vehicle2 (ego)
    _near_coll = partial(P.relSet, vehicle2, "near_coll")
    _super_near  = partial(P.union, _near_coll,   partial(P.relSet, vehicle2, "super_near"))
    _very_near   = partial(P.union, _super_near,  partial(P.relSet, vehicle2, "very_near"))
    _near        = partial(P.union, _very_near,   partial(P.relSet, vehicle2, "near"))
    _visible     = partial(P.union, _near,        partial(P.relSet, vehicle2, "visible"))

    # Exclusive tiers
    tier_near_coll = _near_coll
    tier_super_near  = partial(P.difference, _super_near, _near_coll)
    tier_very_near   = partial(P.difference, _very_near,  _super_near)
    tier_near        = partial(P.difference, _near,       _very_near)
    tier_visible     = partial(P.difference, _visible,    _near)

    # Mutable closure: incumbent/challenger debounce state
    state = {
        "incumbent_name": None,   # currently selected lead vehicle node name
        "challenger_name": None,  # vehicle trying to displace incumbent
        "challenger_count": 0,    # consecutive frames challenger has been raw_best
        "hold_count": 0,          # frames remaining in post-switch freeze
        "last_frame_key": None,   # frozenset of lane vehicle names — frame detector
    }

    def _raw_best_name(lane_tiers, ego_node):
        """Return the name of the nearest same-lane vehicle (tier + Euclidean), or None."""
        best_tier = None
        for lt in lane_tiers:
            if lt:
                best_tier = lt
                break
        if best_tier is None:
            return None
        if len(best_tier) == 1:
            return next(iter(best_tier)).name
        if ego_node is None:
            return next(iter(best_tier)).name  # no position data; pick arbitrary
        ex = ego_node.attr.get("position_x", None)
        ey = ego_node.attr.get("position_y", None)
        if ex is None or ey is None:
            return next(iter(best_tier)).name
        def _sq_dist(n):
            nx = n.attr.get("position_x")
            ny = n.attr.get("position_y")
            if nx is None or ny is None:
                return float("inf")
            return (nx - ex) ** 2 + (ny - ey) ** 2

        best_node = min(best_tier, key=_sq_dist)
        return best_node.name

    def _is_nearest_in_lane(v1_nodes, ego_nodes, lane_vehicles,
                             t_near_coll, t_super_near,
                             t_very_near, t_near, t_visible):
        if not isinstance(v1_nodes, set) or len(v1_nodes) != 1:
            return False
        if not isinstance(lane_vehicles, set):
            return False

        v1_node = next(iter(v1_nodes))

        # Frame detection: update state at most once per frame
        frame_key = frozenset(n.name for n in lane_vehicles)

        if frame_key != state["last_frame_key"]:
            state["last_frame_key"] = frame_key

            tiers = [t_near_coll, t_super_near, t_very_near, t_near, t_visible]
            lane_tiers = [
                (tier & lane_vehicles) if isinstance(tier, set) else set()
                for tier in tiers
            ]

            ego_node = next(iter(ego_nodes)) if (
                isinstance(ego_nodes, set) and len(ego_nodes) == 1
            ) else None

            raw_best = _raw_best_name(lane_tiers, ego_node)

            # State machine update
            if raw_best is None:
                state["incumbent_name"] = None
                state["challenger_name"] = None
                state["challenger_count"] = 0
                state["hold_count"] = 0
            elif state["incumbent_name"] is None:
                state["incumbent_name"] = raw_best
                state["challenger_name"] = None
                state["challenger_count"] = 0
            elif raw_best == state["incumbent_name"]:
                state["challenger_name"] = None
                state["challenger_count"] = 0
                if state["hold_count"] > 0:
                    state["hold_count"] -= 1
            elif state["hold_count"] > 0:
                state["hold_count"] -= 1
            elif raw_best == state["challenger_name"]:
                state["challenger_count"] += 1
                if state["challenger_count"] >= switch_frames:
                    state["incumbent_name"] = raw_best
                    state["challenger_name"] = None
                    state["challenger_count"] = 0
                    state["hold_count"] = hold_frames
            else:
                state["challenger_name"] = raw_best
                state["challenger_count"] = 1

        return v1_node.name == state["incumbent_name"]

    return partial(P.logic_and,
                   same_lane(vehicle1, vehicle2),
                   partial(_is_nearest_in_lane,
                           vehicle1,
                           vehicle2,
                           ego_lane_vehicles,
                           tier_near_coll, tier_super_near,
                           tier_very_near, tier_near, tier_visible))

def kinematic_nearest_same_lane(vehicle1, vehicle2):
    """
    True iff vehicle1 is in the same lane as vehicle2 AND is the nearest
    vehicle ahead of vehicle2 in that lane.

    Strategy: use a kinematic closure that receives three already-resolved
    node sets — v1_nodes, ego_nodes, and all_vehicles (the full vehicle set
    from "G") — and checks whether any other vehicle lies between ego and v1
    along ego's heading axis.

    all_vehicles is passed as a partial(filterByAttr, "G", ...) so that
    __evaluate_predicate resolves it to a real node set before our function
    is called. This avoids any need to access sg directly.
    """

    # Resolved at evaluation time to the set of all vehicle nodes in the scene
    all_vehicles = partial(P.filterByAttr, "G", "base_class",
                           lambda x: x in VEHICLE_CLASSES)

    def _is_nearest_ahead(v1_nodes, ego_nodes, all_vehicle_nodes):
        """
        Returns True iff v1 is the closest vehicle ahead of ego (by longitudinal
        projection onto ego's heading) among all vehicles in the scene.
        """
        if not isinstance(v1_nodes, set) or len(v1_nodes) != 1:
            return False
        if not isinstance(ego_nodes, set) or len(ego_nodes) != 1:
            return False

        v1_node  = next(iter(v1_nodes))
        ego_node = next(iter(ego_nodes))

        ex   = ego_node.attr.get("position_x", None)
        ey   = ego_node.attr.get("position_y", None)
        eyaw = ego_node.attr.get("yaw",        None)
        v1x  = v1_node.attr.get("position_x",  None)
        v1y  = v1_node.attr.get("position_y",  None)

        if any(v is None for v in [ex, ey, eyaw, v1x, v1y]):
            return False

        hx = math.cos(eyaw)
        hy = math.sin(eyaw)

        def longitudinal(cx, cy):
            return hx * (cx - ex) + hy * (cy - ey)

        v1_long = longitudinal(v1x, v1y)
        if v1_long <= 0:
            return False  # v1 is not ahead of ego

        # Check every other vehicle: if any is ahead of ego AND closer than v1,
        # then v1 is not the nearest
        if not isinstance(all_vehicle_nodes, set):
            return False
        for node in all_vehicle_nodes:
            if node is ego_node or node is v1_node:
                continue
            if node.is_phantom():
                continue
            cx = node.attr.get("position_x", None)
            cy = node.attr.get("position_y", None)
            if cx is None or cy is None:
                continue
            c_long = longitudinal(cx, cy)
            if 0 < c_long < v1_long:
                return False  # a closer vehicle exists ahead of ego

        return True

    return partial(P.logic_and,
                   same_lane(vehicle1, vehicle2),
                   partial(_is_nearest_ahead, vehicle1, vehicle2, all_vehicles))


def similar_heading(vehicle1, vehicle2, threshold_rad=math.pi/4):
    """
    AP: vehicle1 and vehicle2 have headings (yaw) within threshold_rad of each other.
    """
    def _heading_check(v1_nodes, v2_nodes, thresh):
        v1_yaw = P.getAttr(v1_nodes, "yaw")
        v2_yaw = P.getAttr(v2_nodes, "yaw")
        if v1_yaw is None or v2_yaw is None:
            return False
        diff = abs(v1_yaw - v2_yaw) % (2 * math.pi)
        diff = min(diff, 2 * math.pi - diff)
        return diff <= thresh
    return partial(_heading_check, vehicle1, vehicle2, threshold_rad)


def moving_in_same_direction_AP(ego, vehicle1):
    """Returns a partial evaluating if two vehicles are moving in same direction"""
    # return partial(P.logic_and, #this is correct, no need to fix.
    #                partial(P.logic_and,
    #                        is_moving_video(ego),
    #                        is_moving_video(vehicle1)),
    #                similar_heading(ego, vehicle1))

    return partial(P.logic_and, is_moving_video(ego), similar_heading(ego, vehicle1))

def weather_checking(weather_condition):
    return partial(filterByGraphAttr, "weather", weather_condition)

def long_deceleration_exceeds(entity, threshold):
    """True when longitudinal_acceleration < -threshold (ego is braking hard)."""
    return partial(P.eq, partial(P.size,
                        partial(P.filterByAttr, entity, "acceleration_longitudinal",
                                (lambda a, t=threshold: a is not None and a < -t))), 1)

# def lat_acceleration_exceeds(entity, threshold):
#     """True when longitudinal_acceleration < -threshold (ego is braking hard)."""
#     return partial(P.eq, partial(P.size,
#                         partial(P.filterByAttr, entity, "acceleration_lateral",
#                                 (lambda a, t=threshold: a is not None and a < -t))), 1)

def abs_accel_exceeds(entity, threshold):
    """True when longitudinal_acceleration < -threshold (ego is braking hard)."""
    return partial(P.eq, partial(P.size,
                        partial(P.filterByAttr, entity, "acceleration",
                                (lambda a, t=threshold: a is not None and a > t))), 1)

def following_predicate(ego, vehicle, follow_distance):
    """Returns a partial evaluating if ego is following vehicle:
       is_too_close AND same_lane AND behind AND is_moving_ego AND is_moving_otherveh"""
    return partial(P.logic_and,
                    partial(P.logic_and,
                           partial(P.logic_and,
                                   partial(P.logic_and,
                                           is_too_close(ego, vehicle, follow_distance),
                                           same_lane(ego, vehicle)),
                                   behind_kinematic(ego, vehicle)),
                           is_moving_video(ego)),
                   is_moving_video(vehicle))

def no_following(follow_duration, follow_distance):
    no_following = SymbolicProperty(
        f"816_vehicle2_cannot_follow_vehicle1_{follow_duration}_{follow_distance}",
        f"(~(too_close & same_lane & behind & is_moving) & X(too_close & same_lane & behind & is_moving)) -> X(~ $[{follow_duration}][too_close & same_lane & behind & is_moving])",
        [("too_close", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind(EGO, YIELD_VEHICLE1)),
                    ("is_moving", is_moving_carla(EGO)),],
    [YIELD_VEHICLE1])
    return no_following

### START OF 2846 SCENARIOS ###

def _2846_S2_nontemporal(follow_distance):
    no_following = SymbolicProperty(
        f"2846_S2_nontemporal_{follow_distance}",
        f"(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving_carla(EGO)),
                    ("is_moving_otherveh", is_moving_carla(YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return no_following

def veh_in_right_lane(vehicle1):
    right_lane = partial(P.filterByAttr, "G", "name", "Right Lane")
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), right_lane))

# note that ego is always in middle lane
def veh_in_middle_lane(vehicle1):
    middle_lane = partial(P.filterByAttr, "G", "name", "Middle Lane")
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), middle_lane))

def veh_in_left_lane(vehicle1):
    left_lane = partial(P.filterByAttr, "G", "name", "Left Lane")
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), left_lane))

"""prop 3 for logical cutin (deceleration beyond a threshold)"""
# def lead_veh_braking_more_than(ego, vehicle, threshold):
#     return partial(P.logic_and,
#                    direct_behind(ego, vehicle),
#                    partial(P.logic_and,
#                    veh_in_middle_lane(vehicle),
#                    abs_accel_exceeds(ego, threshold)))

# def veh_enters_ego_lane(): # Symbolic Property
#     prop = SymbolicProperty(
#         f"veh_enters_ego_lane",
#         f"(veh_in_adj_right_lane U (veh_straddling_ego_lane & X(~veh_straddling_ego_lane)))",
#         [("veh_in_adj_right_lane", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
#          ("veh_straddling_ego_lane", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1))],
#     [YIELD_VEHICLE1])
#     return prop

# def veh_enters_ego_lane(): # Detects the last frame of scenario.
#     prop = SymbolicProperty(
#         f"veh_enters_ego_lane",
#         f"p U ( q & X( ~p & ~q ) )",
#         [("p", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
#          ("q", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1))],
#     [YIELD_VEHICLE1])
#     return prop

# def veh_enters_ego_lane(): # This one works from start to end.
#     prop = SymbolicProperty(
#         f"veh_enters_ego_lane",
#         f"p U ( q & X( q U (~p & ~q) ) )",
#         [("p", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
#          ("q", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1))],
#     [YIELD_VEHICLE1])
#     return prop


# def cut_in_from_right_weather(weather_cond):
#     if weather_cond == "sun/clear":
#         weather_name = "sunny"
#     else:
#         weather_name = weather_cond

#     prop = SymbolicProperty(
#         f"cut_in_from_right_{weather_name}",
#         f"p1 & (p1 U (p2 & (p2 U (p3 & X(p3 U (p3 & X ~p3)))))) & weather", # this gets us from start to end. (start being beginning of the cutin, (last frame of p1).)
#         [("p1", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
#          ("p2", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1)),
#          ("p3", lead_veh_too_close(EGO, YIELD_VEHICLE1, "near_coll")),
#          ("weather", weather_checking(weather_cond))],
#     [YIELD_VEHICLE1])
#     return prop

# """Logical Scenario Extraction"""
# def cut_in_from_right_veh_decel(threshold):
#     prop = SymbolicProperty(
#         f"cut_in_from_right_logical_veh_decel_exceeds_{threshold}",
#         f"p1 & (p1 U (p2 & (p2 U (p3 & X(p3 U (p3 & X ~p3)))))) & F veh_decel_exceeds", # this gets us from start to end. (start being beginning of the cutin, (last frame of p1).)
#         [("p1", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
#          ("p2", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1)),
#          ("p3", lead_veh_too_close(EGO, YIELD_VEHICLE1, "near_coll")),
#          ("veh_decel_exceeds", long_deceleration_exceeds(YIELD_VEHICLE1, threshold))],
#     [YIELD_VEHICLE1])
#     return prop

# def cut_in_from_right_ego_decel(threshold):
#     prop = SymbolicProperty(
#         f"cut_in_from_right_logical_ego_decel_exceeds_{threshold}",
#         f"p1 & (p1 U (p2 & (p2 U (p3 & X(p3 U (p3 & X ~p3)))))) & F veh_decel_exceeds", # this gets us from start to end. (start being beginning of the cutin, (last frame of p1).)
#         [("p1", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
#          ("p2", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1)),
#          ("p3", lead_veh_too_close(EGO, YIELD_VEHICLE1, "near_coll")),
#          ("veh_decel_exceeds", long_deceleration_exceeds(EGO, threshold))],
#     [YIELD_VEHICLE1])
#     return prop

# def is_sunny():
#     prop = SymbolicProperty(
#         f"is_sunny",
#         f"is_sunny",
#         [("is_sunny", weather_checking("sun/clear")),],
#     [YIELD_VEHICLE1])
#     return prop


"""=======================================
START OF RS2V SCENARIO DEFINITIONS
======================================="""
# ROADSCENE2VEC SCENARIO DEFINITIONS
def _2846_S2_nontemporal_RS2V(follow_distance):
    no_following = SymbolicProperty(
        f"2846_S2_nontemporal_{follow_distance}",
        f"(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind_kinematic(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving_video(EGO)),
                    ("is_moving_otherveh", is_moving_video(YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return no_following

def long_following_abstract(follow_distance):
    prop = SymbolicProperty(
        f"long_following_{follow_distance}",
        f" (within_distance & same_lane & behind & is_moving_ego) \
            U ((within_distance & same_lane & behind & is_moving_ego) & X(~(within_distance & same_lane & behind & is_moving_ego)))",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", nearest_same_lane(YIELD_VEHICLE1, EGO)),
                    ("behind", behind_kinematic(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving_video(EGO)),],
                    # ("is_moving_otherveh", is_moving_video(YIELD_VEHICLE1)),],
        [YIELD_VEHICLE1])
    return prop


# f"(~(too_close & same_lane & behind & is_moving) & X(too_close & same_lane & behind & is_moving)) -> X(~ $[{follow_duration}][too_close & same_lane & behind & is_moving])",

# def long_following_abstract_hysteresis(follow_distance, follow_duration,
#                                         same_lane_switch_frames=3, same_lane_hold_frames=1,
#                                         within_dist_hold_frames=1):
#     """
#     Long-following property with two independent layers of noise suppression:

#     AP-level debouncing (noisy predicate smoothing):
#       - within_distance: holds True for up to `within_dist_hold_frames` consecutive
#         False frames before actually going False (tolerates brief distance dropouts).
#       - same_lane: requires a challenger vehicle to be nearest for
#         `same_lane_switch_frames` consecutive frames before displacing the
#         incumbent, then freezes for `same_lane_hold_frames` frames.

#     LTL-level minimum-duration gate:
#       - `$[follow_duration][f]` requires f to hold for at least `follow_duration`
#         frames from the episode start, filtering genuinely short episodes.
#     """
#     prop = SymbolicProperty(
#         f"long_following_{follow_distance}_{follow_duration}",
#         f"$[{follow_duration}][within_distance & same_lane & behind & is_moving_ego] & "
#         f"((within_distance & same_lane & behind & is_moving_ego) "
#         f"U ((within_distance & same_lane & behind & is_moving_ego) "
#         f"& X(~(within_distance & same_lane & behind & is_moving_ego))))",
#         [("within_distance", debounced_is_too_close(EGO, YIELD_VEHICLE1, follow_distance,
#                                                      hold_frames=within_dist_hold_frames)),
#          ("same_lane",       nearest_same_lane(YIELD_VEHICLE1, EGO,
#                                                switch_frames=same_lane_switch_frames,
#                                                hold_frames=same_lane_hold_frames)),
#          ("behind",          behind_kinematic(EGO, YIELD_VEHICLE1)),
#          ("is_moving_ego",   is_moving_video(EGO)),],
#         [YIELD_VEHICLE1])
#     return prop

# def following_nontemporal(follow_distance):
#     no_following = SymbolicProperty(
#         f"2846_S2_nontemporal_{follow_distance}",
#         f"(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)",
#         [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
#                     ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
#                     ("behind", behind_kinematic(EGO, YIELD_VEHICLE1)),
#                     ("is_moving_ego", is_moving_video(EGO)),],
#     [YIELD_VEHICLE1])
#     return no_following

"""
================================================================================
LONGITUDINAL FOLLOWING FOR PAPER
================================================================================
"""

def long_following_abstract_debounce(follow_distance, follow_duration, end_hold_frames=2,
                                        same_lane_switch_frames=2, same_lane_hold_frames=1,  # changed same_lane_switch_Frames from 3 to 2.
                                        within_dist_hold_frames=1):
    """
    Long-following property with two independent layers of noise suppression:

    AP-level debouncing (noisy predicate smoothing):
      - within_distance: holds True for up to `within_dist_hold_frames` consecutive
        False frames before actually going False (tolerates brief distance dropouts).
      - same_lane: requires a challenger vehicle to be nearest for
        `same_lane_switch_frames` consecutive frames before displacing the
        incumbent, then freezes for `same_lane_hold_frames` frames.

    LTL-level minimum-duration gate:
      - `$[follow_duration][f]` requires f to hold for at least `follow_duration`
        frames from the episode start, filtering genuinely short episodes.
    """
    prop = SymbolicProperty(
        f"long_following_{follow_distance}_{follow_duration}",
        f"$[{follow_duration}][within_distance & same_lane & behind & is_moving_ego] & "
        f"((within_distance & same_lane & behind & is_moving_ego) "
        f"U ((within_distance & same_lane & behind & is_moving_ego) "
        f"& X($[{end_hold_frames}][~(within_distance & same_lane & behind & is_moving_ego)])))",
        [("within_distance", debounced_is_too_close(EGO, YIELD_VEHICLE1, follow_distance,
                                                     hold_frames=within_dist_hold_frames)),
         ("same_lane",       nearest_same_lane(YIELD_VEHICLE1, EGO,
                                               switch_frames=same_lane_switch_frames,
                                               hold_frames=same_lane_hold_frames)),
         ("behind",          behind_kinematic(EGO, YIELD_VEHICLE1)),
         ("is_moving_ego",   is_moving_video(EGO)),],
        [YIELD_VEHICLE1])
    return prop


# def long_following_abstract_headway_debounce(max_headway_s, follow_duration, end_hold_frames=2,
#                                               same_lane_switch_frames=2, same_lane_hold_frames=1,
#                                               within_dist_hold_frames=1):
#     """
#     Like long_following_abstract_debounce but uses a headway (time-gap) gate
#     instead of a distance-category gate, matching the definition in extract_aps.py.

#         headway = longitudinal_distance / ego_speed  [seconds]

#     The vehicle qualifies when headway <= max_headway_s.  At standstill (ego speed
#     <= min_ego_speed) no vehicle qualifies — identical to extract_aps.py behaviour.

#     Debouncing parameters are the same as long_following_abstract_debounce.
#     """
#     prop = SymbolicProperty(
#         f"long_following_headway_{max_headway_s}s_{follow_duration}",
#         f"$[{follow_duration}][within_headway & same_lane & behind & is_moving_ego] & "
#         f"((within_headway & same_lane & behind & is_moving_ego) "
#         f"U ((within_headway & same_lane & behind & is_moving_ego) "
#         f"& X($[{end_hold_frames}][~(within_headway & same_lane & behind & is_moving_ego)])))",
#         [("within_headway", debounced_within_headway(EGO, YIELD_VEHICLE1, max_headway_s,
#                                                       hold_frames=within_dist_hold_frames)),
#          ("same_lane",      nearest_same_lane(YIELD_VEHICLE1, EGO,
#                                               switch_frames=same_lane_switch_frames,
#                                               hold_frames=same_lane_hold_frames)),
#          ("behind",         behind_kinematic(EGO, YIELD_VEHICLE1)),
#          ("is_moving_ego",  is_moving_video(EGO)),],
#         [YIELD_VEHICLE1])
#     return prop


def long_following_logical_ego_decel(follow_distance,  decel):
    start = SymbolicProperty(
        f"long_following_{follow_distance}_ego_decel_{decel}",
        f" f U (f & X(~f)) & F ((f U (f & X(~f))) & ego_decel)",
        [("f", following_predicate(EGO, YIELD_VEHICLE1, follow_distance)),
         ("ego_decel", long_deceleration_exceeds(EGO, decel)),],
        [YIELD_VEHICLE1])
    return start

def long_following_logical_veh_decel(follow_distance,  decel):
    start = SymbolicProperty(
        f"long_following_{follow_distance}_veh_decel_{decel}",
        f" f U (f & X(~f)) & F ((f U (f & X(~f))) & veh_decel)",
        [("f", following_predicate(EGO, YIELD_VEHICLE1, follow_distance)),
         ("veh_decel", long_deceleration_exceeds(YIELD_VEHICLE1, decel)),],
        [YIELD_VEHICLE1])
    return start

"""
================================================================================
CUTIN FOR PAPER
================================================================================
"""
"""
APs for cut-in scenario
"""
"""PROP 1 for CUTIN"""
def veh_in_adj_right_lane(ego, vehicle):
    return partial(P.logic_and,
                    moving_in_same_direction_AP(ego, vehicle),
                    partial(P.logic_and,
                    veh_in_right_lane(vehicle),
                    is_too_close(ego, vehicle, "visible")))

"""Prop 2 for cut-in"""
def veh_straddling_ego_lane(ego, vehicle):
    return partial(P.logic_and,
                   side_behind(ego, vehicle),
                   partial(P.logic_and,
                   veh_in_middle_lane(vehicle),
                   partial(P.logic_and,
                   is_too_close(ego, vehicle, "visible"),
                   lateral_velocity_leftward(ego, vehicle))))


"""prop 3 for cutin"""
def lead_veh_too_close(ego, vehicle, dist):
    return partial(P.logic_and,
                   direct_behind(ego, vehicle),
                   partial(P.logic_and,
                   veh_in_middle_lane(vehicle),
                   is_too_close(ego, vehicle, dist)))


def veh_in_adj_right_lane_AP(): 
    prop = SymbolicProperty(
        f"veh_in_adj_right_lane",
        f"p U (p & X(~p))",
        [("p", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return prop

def veh_straddling_ego_lane_AP(): 
    prop = SymbolicProperty(
        f"veh_straddling_ego_lane",
        f"p U (p & X(~p))",
        [("p", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return prop

def lead_veh_too_close_AP(): 
    prop = SymbolicProperty(
        f"lead_veh_too_close",
        f"p U (p & X(~p))",
        [("p",  lead_veh_too_close(EGO, YIELD_VEHICLE1, "very_near")),],
    [YIELD_VEHICLE1])
    return prop


# "(p1 & ~p2 & ~p3) U ( p2 & X( (p2 & ~p3) U p3 ) )" -> detects 4 frames where p3 has become true. all 4 are valid but I don't want it to detect 4 of same scenairo.
# "(p1 & ~p2 & ~p3) U ( p2 & X( (p2 & ~p3) U  ( p3 & X( p3 U (p3 & X ~p3) ) )))" -> detects from last frame of p1, to when p3 becomes false at end of scenario.
# (p1) U ( p2 & X( (p2) U  ( p3 & X( p3 U (p3 & X ~p3) ) ))) -> does same as  above. morecompact.
# Critical  detail: above doesn't require p1 to be true ever.
#  correct: p1 & (p1 U (p2 & (p2 U (p3 & X(p3 U (p3 & X ~p3))))))
def cut_in_from_right():
    prop = SymbolicProperty(
        f"cut_in_from_right",
        f"p1 & (p1 U (p2 & (p2 U (p3 & X(p3 U (p3 & X ~p3))))))", # this gets us from start to end. (start being beginning of the cutin, (last frame of p1).)
        [("p1", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
         ("p2", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1)),
         ("p3", lead_veh_too_close(EGO, YIELD_VEHICLE1, "very_near")),],
    [YIELD_VEHICLE1])
    return prop

"""
End Cut-in scenario
"""

# def long_following_logical_veh_decel(follow_distance, decel):
#     start = SymbolicProperty(
#         f"long_following_{follow_distance}_veh_decel_{decel}",
#         f" f U (f & X(~f)) & F veh_decel",
#         [("f", following_predicate(EGO, YIELD_VEHICLE1, follow_distance)),
#          ("veh_decel", long_deceleration_exceeds(YIELD_VEHICLE1, decel)),],
#         [YIELD_VEHICLE1])
#     return start

# def long_following_logical_ego_decel(follow_distance):
#     start = SymbolicProperty(
#         f"2846_S2_full_body_U_end_{follow_distance}",
#         f"(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) \
#                   U ((within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) & X(~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)))",
#         [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
#                     ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
#                     ("behind", behind_kinematic(EGO, YIELD_VEHICLE1)),
#                     ("is_moving_ego", is_moving_video(EGO)),
#                     ("is_moving_otherveh", is_moving_video(YIELD_VEHICLE1)),],
#         [YIELD_VEHICLE1])
#     return start

# def cut_in_right_rs2v(follow_distance):
#     start = SymbolicProperty(
#         f"2846_S2_full_body_U_end_{follow_distance}",
#         f"(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) \
#                   U ((within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) & X(~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)))",
#         [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
#                     ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
#                     ("behind", behind(EGO, YIELD_VEHICLE1)),
#                     ("is_moving_ego", is_moving_video(EGO)),
#                     ("is_moving_otherveh", is_moving_video(YIELD_VEHICLE1)),],
#         [YIELD_VEHICLE1])
#     return start



"""=======================================
END OF RS2V SCENARIO DEFINITIONS
======================================="""

# P = (within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)

"""
def _2846_S2_start(follow_distance):
    start = SymbolicProperty(
        f"2846_S2_start_{follow_distance}",
        f"~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) & X(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_otherveh", is_moving(YIELD_VEHICLE1)),],
        [YIELD_VEHICLE1])
    return start

def _2846_S2_full_body_U_end(follow_distance):
    start = SymbolicProperty(
        f"2846_S2_full_body_U_end_{follow_distance}",
        f"(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) \
                  U ((within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) & X(~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)))",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_otherveh", is_moving(YIELD_VEHICLE1)),],
        [YIELD_VEHICLE1])
    return start

def _2846_S2_full_start_U_body_U_end(follow_distance):
    start = SymbolicProperty(
        f"2846_S2_full_start_U_body_U_end_{follow_distance}",
        f"(~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) & X(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)) \
            U (within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) \
                  U ((within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh) & X(~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)))",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_otherveh", is_moving(YIELD_VEHICLE1)),],
        [YIELD_VEHICLE1])
    return start
"""

"""SCENARIO 1: Lateral Side-by-Side driving"""
"""
def _2846_S1_nontemporal(follow_distance):
    lateral = SymbolicProperty(
        f"2846_S1_nontemporal_{follow_distance}",
        f"(within_distance & beside & is_moving_ego & is_moving_otherveh)",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("beside", beside(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_otherveh", is_moving(YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return lateral 

def _2846_S1_rising_edge(follow_distance):
    lateral = SymbolicProperty(
        f"2846_S1_rising_edge_{follow_distance}",
        f"~(within_distance & beside & is_moving_ego & is_moving_otherveh) & X(within_distance & beside & is_moving_ego & is_moving_otherveh)",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("beside", beside(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_otherveh", is_moving(YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return lateral

def _2846_S1_full_body_U_end(follow_distance):
    lateral = SymbolicProperty(
        f"2846_S1_full_body_U_end{follow_distance}",
        f"(within_distance & beside & is_moving_ego & is_moving_otherveh) \
            U ((within_distance & beside & is_moving_ego & is_moving_otherveh) & X(~(within_distance & beside & is_moving_ego & is_moving_otherveh)))",
        [("within_distance", is_too_close(EGO, YIELD_VEHICLE1, follow_distance)),
                    ("beside", beside(EGO, YIELD_VEHICLE1)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_otherveh", is_moving(YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return lateral



def _2846_S1US2_nontemporal(follow_distance, side_distance):
    LEAD_VEHICLE = YIELD_VEHICLE1
    LATERAL_VEHICLE = YIELD_VEHICLE2
    lateral = SymbolicProperty(
        f"2846_S1US2_nontemporal_{follow_distance}",
        f"~(within_distance_lateral & beside & is_moving_ego & is_moving_side_veh & within_distance_longtdl & same_lane & behind & is_moving_front_veh)", #~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)
        [("within_distance_lateral", is_too_close(EGO, LATERAL_VEHICLE, side_distance)),
                    ("beside", beside(EGO, LATERAL_VEHICLE)),
                    ("is_moving_ego", is_moving(EGO)),
                    ("is_moving_side_veh", is_moving(LATERAL_VEHICLE)),
                    ("within_distance_longtdl", is_too_close(EGO, LEAD_VEHICLE, follow_distance)),
                    ("same_lane", same_lane(EGO, LEAD_VEHICLE)),
                    ("behind", behind(EGO, LEAD_VEHICLE)),
                    ("is_moving_front_veh", is_moving(LEAD_VEHICLE)),],
    [YIELD_VEHICLE1, YIELD_VEHICLE2])
    return lateral

# Using T = 2
# P_s = (X $[{follow_duration}][(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)])
# P_e = (X $[{follow_duration}][~(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)])

# P_s = (X(within_distance & same_lane & behind & is_moving_ego & is_moving_otherveh)])
"""

def no_following_emergency(follow_duration):
    no_following_emergency = SymbolicProperty(
        f"921_vehicle2_cannot_follow_emergency_vehicle1_{follow_duration}",
        f"(~(too_close & same_lane & behind & v1_emergency) & X(too_close & same_lane & behind & v1_emergency)) -> X(~ $[{follow_duration}][too_close & same_lane & behind & v1_emergency])",
        [("too_close", is_too_close_emergency(EGO, YIELD_VEHICLE1)),
                    ("same_lane", same_lane(EGO, YIELD_VEHICLE1)),
                    ("behind", behind(EGO, YIELD_VEHICLE1)),
                    ("v1_emergency", is_emergency_vehicle(YIELD_VEHICLE1)),],
    [YIELD_VEHICLE1])
    return no_following_emergency


LANE_VEHICLE1 = SymbolicEntity('lane_vehicle_1', VEHICLE_CLASSES)
LANE1 = SymbolicEntity('lane_1', ['lane'])
LANE2 = SymbolicEntity('lane_2', ['lane'])


def only_in_lane(vehicle, lane):
    """True iff the vehicle is in the lane and no other lanes"""
    return partial(P.logic_and, partial(P.eq, partial(P.size, partial(P.symmetric_difference, entity_lanes(vehicle), lane)), 0),
                   partial(P.defined, lane))

def in_lane(vehicle, lane):
    """True iff the vehicle is in the lane"""
    return non_empty(partial(P.intersection, lane, entity_lanes(vehicle)))


def lane_in_junction(lane):
    return partial(P.gt, partial(P.size, partial(P.relSet, partial(P.relSet, lane, "isIn"), "isIn")), 0)


def is_in_a_junction(vehicle):
    return partial(P.gt, partial(P.size, entity_junctions(vehicle)), 0)


def only_in_junction(vehicle):
    not_out_of_junction = partial(P.eq, partial(P.size, partial(P.difference, entity_roads(vehicle), partial(P.relSet, entity_junctions(vehicle), "isIn", edge_type="incoming"))), 0)
    return partial(P.logic_and, is_in_a_junction(vehicle), not_out_of_junction)


def lanes_to_right(lane):
    return partial(P.relSet, lane, 'toRightOf', edge_type="incoming")

def rightmost(lane):
    return partial(P.eq, partial(P.size, lanes_to_right(lane)), 0)


def lanes_to_left(lane):
    return partial(P.difference,
                        partial(P.relSet, lane, 'toLeftOf', edge_type="incoming"),
                        partial(P.relSet, lane, 'opposes', edge_type="incoming"))

def leftmost(lane):
    # lanes to the left \ opposing lanes
    return partial(P.eq, partial(P.size, lanes_to_left(lane)), 0)

def lanes_match(lane1, lane2):
    # if right:
    l1_rightmost = rightmost(lane1)
    l2_rightmost = rightmost(lane2)
    rightmost_equal = partial(P.boolean_equals, l1_rightmost, l2_rightmost)
        # return rightmost_equal
    # else:
        # lanes to the left \ opposing lanes
    l1_leftmost = leftmost(lane1)
    l2_leftmost = leftmost(lane2)
    leftmost_equal = partial(P.boolean_equals, l1_leftmost, l2_leftmost)
        # return leftmost_equal
    return partial(P.logic_and, rightmost_equal, leftmost_equal)


def two_lane_road(lane):
    lane_to_left = lanes_to_left(lane)
    two_to_left = lanes_to_left(lane_to_left)

    lane_to_right = lanes_to_right(lane)
    two_to_right = lanes_to_right(lane_to_right)

    two_lane_right = partial(P.logic_and,
                             partial(P.logic_and,
                                     set_size_eq(lane_to_right, 0),
                                     set_size_eq(lane_to_left, 1)),
                             set_size_eq(two_to_left, 0)
                             )
    two_lane_left = partial(P.logic_and,
                             partial(P.logic_and,
                                     set_size_eq(lane_to_left, 0),
                                     set_size_eq(lane_to_right, 1)),
                             set_size_eq(two_to_right, 0)
                             )
    return partial(P.logic_or, two_lane_right, two_lane_left)



only_in_lane2 = partial(P.ite,
                        partial(P.defined, LANE2),
                        # partial(P.logic_implies, two_lane_road(LANE2),
                        # partial(P.logic_and,
                        #         partial(P.logic_not, lane_in_junction(LANE2)),
                        #                 only_in_lane(LANE_VEHICLE1, LANE2)))
                partial(P.logic_and,
                        partial(P.logic_and, two_lane_road(LANE2), partial(P.logic_not, lane_in_junction(LANE2))),
                        only_in_lane(EGO, LANE2))
                        , False)


def lane_arity_matches():
    lane_matches = SymbolicProperty(f"846_lane_you_leave_must_match_lane_you_enter",
        # "((only_in_lane1 & X(only_in_junction) & X(X(((only_in_junction & !(only_in_lane2)) U (only_in_lane2 | !(only_in_junction)))))) -> (only_in_lane1 & X(only_in_junction) & X(X(((only_in_junction & !(only_in_lane2)) U (only_in_lane2 & lane1_match_lane2))))))",
                                  "((only_in_lane1 & X(only_in_junction) & X(X(((only_in_junction & !(only_in_lane2)) U only_in_lane2)))) -> (only_in_lane1 & X(only_in_junction) & X(X(((only_in_junction & !(only_in_lane2)) U (only_in_lane2 & lane1_match_lane2))))))",
        [("only_in_lane1",
        partial(P.logic_and,
          partial(
             P.logic_and, (partial(P.logic_not, lane_in_junction(LANE1))),
                           only_in_lane(EGO, LANE1)
         ),
                two_lane_road(LANE1))),
         ("only_in_junction", is_in_a_junction(EGO)),
         ("only_in_lane2", only_in_lane2),
         ("lane1_match_lane2", lanes_match(LANE1, LANE2))],
        [LANE1, LANE2])
    return lane_matches


BIKE1 = SymbolicEntity('bike_1', ['bicycle'])

def is_direct_right_of(vehicle1, vehicle2):
    """Returns if vehicle 2 is to the direct right of vehicle 1"""
    # right_v1_entities = partial(P.relSet, vehicle1, "toRightOf", edge_type='incoming')
    # check from our perspective instead
    right_v1_entities = partial(P.relSet, vehicle2, "toRightOf")
    side_v1_entities = partial(P.union, partial(P.relSet, vehicle2, "atSRearOf"),
                                 partial(P.relSet, vehicle2, "inSFrontOf"))
    side_right_v1_entities = partial(P.intersection, right_v1_entities, side_v1_entities)
    return set_size_eq(partial(P.intersection, side_right_v1_entities, vehicle1), 1)

def give_bikes_room(buffer=False):
    give_bikes_room_passing = SymbolicProperty(f"839_give_bikes_room_passing{'_buffer' if buffer else ''}",
       # ((q & !(r) & F((r | !(p)))) -> X((p U r)))
        "((behind_bike & bike_safe_distance & !(in_front_bike) & F((in_front_bike | !(bike_safe_distance)))) -> X((bike_safe_distance U in_front_bike)))",
    # "((behind_bike & !(in_front_bike) & F((in_front_bike | !(bike_safe_distance)))) -> (bike_safe_distance U in_front_bike))",
        [
                   ("behind_bike", partial(P.logic_and, behind(EGO, BIKE1), same_lane(BIKE1, EGO))),
                   ("in_front_bike", partial(P.logic_implies,
                                             same_lane(BIKE1, EGO),
                                             behind(BIKE1, EGO))),
                   ("bike_safe_distance",
                    partial(P.logic_and,
                            partial(P.logic_not, is_too_close_bike(EGO, BIKE1, add_buffer=buffer)),
                            partial(P.logic_not, is_direct_right_of(BIKE1, EGO))
                            )
                   )
                   ],
        [BIKE1])
    return give_bikes_room_passing


ENTITY_BEING_PASSED = SymbolicEntity('entity_being_passed', VEHICLE_CLASSES)


def entities_within(entity, distance):
    if distance == 'safe_hazard':
        return partial(P.relSet, entity, "safe_hazard")
    close_to_entity = partial(P.union, partial(P.relSet, entity, "safe_hazard"),
                                 partial(P.relSet, entity, "near_coll"))
    if distance == 'near_coll':
        return close_to_entity
    close_to_entity = partial(P.union, close_to_entity,
                                 partial(P.relSet, entity, "super_near"))
    if distance == 'super_near':
        return close_to_entity
    close_to_entity = partial(P.union, close_to_entity,
                                 partial(P.relSet, entity, "very_near"))
    if distance == 'very_near':
        return close_to_entity
    close_to_entity = partial(P.union, close_to_entity,
                                 partial(P.relSet, entity, "near"))
    if distance == 'near':
        return close_to_entity
    close_to_entity = partial(P.union, close_to_entity,
                                 partial(P.relSet, entity, "visible"))
    return close_to_entity

def opposing_lane_clear(entity, lane):
    opposing_lane = partial(P.relSet, lane, "opposes")
    opposing_lane_entities = partial(P.relSet, opposing_lane, "isIn", edge_type="incoming")
    entities_to_avoid = partial(P.intersection,
                                entities_within(entity, distance='visible'),
                                opposing_lane_entities)
    # clear if set is empty
    lane_clear = set_size_eq(entities_to_avoid, 0)
    # if we are in the opp lane, it better be clear
    return partial(P.logic_implies, in_lane(entity, opposing_lane), lane_clear)


def observed_entity(entity):
    return set_size_eq(partial(P.filterByAttr, entity, "PHANTOM", lambda x: x), 0)


# Relevant to detect Property 6 from the paper.
opp_clear_for_crossing = SymbolicProperty("843_opp_clear_for_crossing",
   # ((q & !(r) & F((r | !(p)))) -> X((p U r)))
    "((behind_entity & opp_clear & !(in_front_entity) & in_one_lane & F(((in_front_entity & in_one_lane) | !(opp_clear)))) -> X((opp_clear U (in_front_entity & in_one_lane))))",
    [
               ("behind_entity", partial(P.logic_and, behind(EGO, ENTITY_BEING_PASSED), same_lane(ENTITY_BEING_PASSED, EGO))),
               ("in_front_entity", partial(P.logic_implies, observed_entity(ENTITY_BEING_PASSED), partial(P.logic_and, behind(ENTITY_BEING_PASSED, EGO), same_lane(ENTITY_BEING_PASSED, EGO)))),
               ("opp_clear", opposing_lane_clear(EGO, LANE1)),
              ("in_one_lane", partial(
                  P.logic_and, (partial(P.logic_not, lane_in_junction(LANE1))),
                  only_in_lane(EGO, LANE1)
              )),
               ],
    [ENTITY_BEING_PASSED, LANE1])

# all_symbolic_properties = [symbolic_stop_sign_yield, symbolic_stop_sign_tie_left_yield,
#                            symbolic_stop_sign_yield_to_emergency,
#                            # lane_arity_matches(False), lane_arity_matches(True),
#                            lane_arity_matches(),
#                            give_bikes_room(False), give_bikes_room(True),
#                            opp_clear_for_crossing, no_following_emergency(10),
#                            _2846_S2_nontemporal('visible'), _2846_S2_rising_edge('visible'),
#                            #_2846_S2_start_hysteresis(2, 'visible'), # T=2 corresponds to around 1 second.
#                            _2846_S1_nontemporal('super_near'),
#                            _2846_S1_rising_edge('super_near'),
#                            _2846_S1US2_nontemporal('visible', 'super_near'),
#                            _2846_S2('visible')]

# for frames in [10, 50]:
#     for distances in ['visible', 'super_near']:
#         all_symbolic_properties.append(no_following(frames, distances))

"""LONG FOLLOWING SCENARIOS"""
all_symbolic_properties = []
for distances in ['visible', 'near']:
    all_symbolic_properties.append(long_following_abstract_debounce(distances, 2))
    all_symbolic_properties.append(long_following_abstract_debounce(distances, 3))

"""CUT IN SCENARIOS!"""
all_symbolic_properties.append(veh_in_adj_right_lane_AP())
all_symbolic_properties.append(veh_straddling_ego_lane_AP())
all_symbolic_properties.append(lead_veh_too_close_AP())
all_symbolic_properties.append(cut_in_from_right())
    

    # all_symbolic_properties.append(long_fsg_processing/code/tracking_estimation/add_tracks_state_to_sg.pyollowing_abstract_debounce(distances, 5))
    # all_symbolic_properties.append(long_following_abstract_debounce(distances, 10))


# all_symbolic_properties.append(long_following_logical_veh_decel('very_near', 3))
# all_symbolic_properties.append(long_following_logical_veh_decel('super_near', 3))

# all_symbolic_properties.append(long_following_abstract_hysteresis('very_near', 5))
# all_symbolic_properties = [_2846_S2_nontemporal_RS2V('visible'), _2846_S2_full_body_U_end_RS2V('visible'), \
                        #    cut_in_from_right_weather("sun/clear"), cut_in_from_right(), cut_in_from_right_logical(20)]

                        
# all_symbolic_properties = [long_following_abstract('super_near'), long_following_logical_ego_decel('super_near', 3), long_following_logical_veh_decel('super_near', 3)]
# all_symbolic_properties = [cut_in_from_right()]
# all_symbolic_properties = [long_following_abstract()]
