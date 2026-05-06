import SG_Primitives as P
from functools import partial
import math

from SymbolicEntity import SymbolicEntity
from SymbolicProperty import SymbolicProperty

VEHICLE_CLASSES = ['car', 'truck', 'van', 'bus', 'motorcycle', 'bicycle', 'vehicle']
MOTOR_VEHICLE_CLASSES = ['car', 'truck', 'van', 'bus', 'motorcycle', 'vehicle']
ROAD_USER_CLASSES = ['car', 'truck', 'van', 'bus', 'motorcycle', 'bicycle', 'vehicle', 'person']

# ── Symbolic entities ─────────────────────────────────────────────────────────

EGO = partial(P.filterByAttr, "G", "name", "ego")
YIELD_VEHICLE1 = SymbolicEntity('yield_vehicle_1', VEHICLE_CLASSES)

# ── Low-level helpers ─────────────────────────────────────────────────────────

def entity_lanes(entity):
    return partial(P.relSet, entity, "isIn")

def set_size_eq(nodes, size):
    return partial(P.eq, partial(P.size, nodes), size)

def non_empty(nodes):
    return partial(P.gt, partial(P.size, nodes), 0)

def same_lane(vehicle1, vehicle2):
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), entity_lanes(vehicle2)))

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

def is_too_close(vehicle2, vehicle1, distance):
    close_to_vehicle_2 = entities_within(vehicle2, distance)
    return set_size_eq(partial(P.intersection, close_to_vehicle_2, vehicle1), 1)

def debounced_is_too_close(vehicle2, vehicle1, distance, hold_frames=1):
    """
    Like is_too_close but tolerates up to `hold_frames` consecutive False frames
    before actually going False.
    """
    close_to_v2 = entities_within(vehicle2, distance)
    intersection_set = partial(P.intersection, close_to_v2, vehicle1)

    hold_state = {}
    output_state = {}

    def _held(v2_nodes, v1_nodes, intersection):
        raw = isinstance(intersection, set) and len(intersection) == 1

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

def nearest_same_lane(vehicle1, vehicle2, switch_frames=1, hold_frames=0):
    """
    True iff vehicle1 is in the same lane as vehicle2 AND is the debounced
    nearest same-lane vehicle to ego by Euclidean distance.
    """
    ego_lane_vehicles = partial(P.relSet, entity_lanes(vehicle2), "isIn", edge_type="incoming")

    _near_coll  = partial(P.relSet, vehicle2, "near_coll")
    _super_near = partial(P.union, _near_coll,   partial(P.relSet, vehicle2, "super_near"))
    _very_near  = partial(P.union, _super_near,  partial(P.relSet, vehicle2, "very_near"))
    _near       = partial(P.union, _very_near,   partial(P.relSet, vehicle2, "near"))
    _visible    = partial(P.union, _near,        partial(P.relSet, vehicle2, "visible"))

    tier_near_coll  = _near_coll
    tier_super_near = partial(P.difference, _super_near, _near_coll)
    tier_very_near  = partial(P.difference, _very_near,  _super_near)
    tier_near       = partial(P.difference, _near,       _very_near)
    tier_visible    = partial(P.difference, _visible,    _near)

    state = {
        "incumbent_name": None,
        "challenger_name": None,
        "challenger_count": 0,
        "hold_count": 0,
        "last_frame_key": None,
    }

    def _raw_best_name(lane_tiers, ego_node):
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
            return next(iter(best_tier)).name
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
        return min(best_tier, key=_sq_dist).name

    def _is_nearest_in_lane(v1_nodes, ego_nodes, lane_vehicles,
                             t_near_coll, t_super_near,
                             t_very_near, t_near, t_visible):
        if not isinstance(v1_nodes, set) or len(v1_nodes) != 1:
            return False
        if not isinstance(lane_vehicles, set):
            return False

        v1_node = next(iter(v1_nodes))
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

def behind_kinematic(vehicle1, vehicle2, longitudinal_threshold=0.0,
                     lateral_tolerance=None, heading_threshold_rad=math.pi / 4):
    def _behind_check(v1_nodes, v2_nodes, long_thresh, lat_tol, head_thresh):
        x1   = P.getAttr(v1_nodes, "position_x")
        y1   = P.getAttr(v1_nodes, "position_y")
        x2   = P.getAttr(v2_nodes, "position_x")
        y2   = P.getAttr(v2_nodes, "position_y")
        yaw1 = P.getAttr(v1_nodes, "yaw")
        yaw2 = P.getAttr(v2_nodes, "yaw")

        if any(v is None for v in [x1, y1, x2, y2, yaw1, yaw2]):
            return False

        yaw_diff = abs(yaw1 - yaw2) % (2 * math.pi)
        yaw_diff = min(yaw_diff, 2 * math.pi - yaw_diff)
        if yaw_diff > head_thresh:
            return False

        dx = x1 - x2
        dy = y1 - y2
        hx = math.cos(yaw2)
        hy = math.sin(yaw2)
        longitudinal = hx * dx + hy * dy
        behind_check = longitudinal < long_thresh

        if lat_tol is not None:
            lx, ly = -hy, hx
            lateral = abs(lx * dx + ly * dy)
            return behind_check and lateral <= lat_tol

        return behind_check

    return partial(_behind_check, vehicle1, vehicle2,
                   longitudinal_threshold, lateral_tolerance, heading_threshold_rad)

def side_behind(vehicle1, vehicle2):
    behind_v1_entities = partial(P.relSet, vehicle1, "atSRearOf")
    is_behind_v1 = set_size_eq(partial(P.intersection, behind_v1_entities, vehicle2), 1)
    front_v2_entities = partial(P.relSet, vehicle2, "inSFrontOf")
    is_front_v2 = set_size_eq(partial(P.intersection, front_v2_entities, vehicle1), 1)
    return partial(P.logic_and, is_behind_v1, is_front_v2)

def direct_behind(vehicle1, vehicle2):
    behind_v1_entities = partial(P.relSet, vehicle1, "atDRearOf")
    is_behind_v1 = set_size_eq(partial(P.intersection, behind_v1_entities, vehicle2), 1)
    front_v2_entities = partial(P.relSet, vehicle2, "inDFrontOf")
    is_front_v2 = set_size_eq(partial(P.intersection, front_v2_entities, vehicle1), 1)
    return partial(P.logic_and, is_behind_v1, is_front_v2)

def is_moving_video(entity):
    return partial(P.eq, partial(P.size,
                          partial(P.filterByAttr, entity, "velocity",
                                  (lambda a: a is not None and P.gt(a, 0.2)))), 1)

def similar_heading(vehicle1, vehicle2, threshold_rad=math.pi / 4):
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
    return partial(P.logic_and, is_moving_video(ego), similar_heading(ego, vehicle1))

def lateral_velocity_leftward(ego, entity):
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

def long_deceleration_exceeds(entity, threshold):
    return partial(P.eq, partial(P.size,
                        partial(P.filterByAttr, entity, "acceleration_longitudinal",
                                (lambda a, t=threshold: a is not None and a < -t))), 1)

def following_predicate(ego, vehicle, follow_distance):
    return partial(P.logic_and,
                   partial(P.logic_and,
                           partial(P.logic_and,
                                   partial(P.logic_and,
                                           is_too_close(ego, vehicle, follow_distance),
                                           same_lane(ego, vehicle)),
                                   behind_kinematic(ego, vehicle)),
                           is_moving_video(ego)),
                   is_moving_video(vehicle))

# ── Lane helpers ──────────────────────────────────────────────────────────────

def veh_in_right_lane(vehicle1):
    right_lane = partial(P.filterByAttr, "G", "name", "Right Lane")
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), right_lane))

def veh_in_middle_lane(vehicle1):
    middle_lane = partial(P.filterByAttr, "G", "name", "Middle Lane")
    return non_empty(partial(P.intersection, entity_lanes(vehicle1), middle_lane))

# ── Scenario definitions ──────────────────────────────────────────────────────

"""
================================================================================
LONGITUDINAL FOLLOWING
================================================================================
"""

def long_following_abstract_debounce(follow_distance, follow_duration, end_hold_frames=2,
                                     same_lane_switch_frames=2, same_lane_hold_frames=1,
                                     within_dist_hold_frames=1):
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
         ("is_moving_ego",   is_moving_video(EGO))],
        [YIELD_VEHICLE1])
    return prop

def long_following_logical_ego_decel(follow_distance, decel):
    prop = SymbolicProperty(
        f"long_following_{follow_distance}_ego_decel_{decel}",
        f" f U (f & X(~f)) & F ((f U (f & X(~f))) & ego_decel)",
        [("f",         following_predicate(EGO, YIELD_VEHICLE1, follow_distance)),
         ("ego_decel", long_deceleration_exceeds(EGO, decel))],
        [YIELD_VEHICLE1])
    return prop

def long_following_logical_veh_decel(follow_distance, decel):
    prop = SymbolicProperty(
        f"long_following_{follow_distance}_veh_decel_{decel}",
        f" f U (f & X(~f)) & F ((f U (f & X(~f))) & veh_decel)",
        [("f",         following_predicate(EGO, YIELD_VEHICLE1, follow_distance)),
         ("veh_decel", long_deceleration_exceeds(YIELD_VEHICLE1, decel))],
        [YIELD_VEHICLE1])
    return prop

"""
================================================================================
CUT-IN
================================================================================
"""

def veh_in_adj_right_lane(ego, vehicle):
    return partial(P.logic_and,
                   moving_in_same_direction_AP(ego, vehicle),
                   partial(P.logic_and,
                           veh_in_right_lane(vehicle),
                           is_too_close(ego, vehicle, "visible")))

def veh_straddling_ego_lane(ego, vehicle):
    return partial(P.logic_and,
                   side_behind(ego, vehicle),
                   partial(P.logic_and,
                           veh_in_middle_lane(vehicle),
                           partial(P.logic_and,
                                   is_too_close(ego, vehicle, "visible"),
                                   lateral_velocity_leftward(ego, vehicle))))

def lead_veh_too_close(ego, vehicle, dist):
    return partial(P.logic_and,
                   direct_behind(ego, vehicle),
                   partial(P.logic_and,
                           veh_in_middle_lane(vehicle),
                           is_too_close(ego, vehicle, dist)))

def veh_in_adj_right_lane_AP():
    return SymbolicProperty(
        "veh_in_adj_right_lane",
        "p U (p & X(~p))",
        [("p", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1))],
        [YIELD_VEHICLE1])

def veh_straddling_ego_lane_AP():
    return SymbolicProperty(
        "veh_straddling_ego_lane",
        "p U (p & X(~p))",
        [("p", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1))],
        [YIELD_VEHICLE1])

def lead_veh_too_close_AP():
    return SymbolicProperty(
        "lead_veh_too_close",
        "p U (p & X(~p))",
        [("p", lead_veh_too_close(EGO, YIELD_VEHICLE1, "very_near"))],
        [YIELD_VEHICLE1])

def cut_in_from_right():
    return SymbolicProperty(
        "cut_in_from_right",
        "p1 & (p1 U (p2 & (p2 U (p3 & X(p3 U (p3 & X ~p3))))))",
        [("p1", veh_in_adj_right_lane(EGO, YIELD_VEHICLE1)),
         ("p2", veh_straddling_ego_lane(EGO, YIELD_VEHICLE1)),
         ("p3", lead_veh_too_close(EGO, YIELD_VEHICLE1, "very_near"))],
        [YIELD_VEHICLE1])

# ── Active property list ──────────────────────────────────────────────────────

all_symbolic_properties = []

for distances in ['visible', 'near']:
    # all_symbolic_properties.append(long_following_abstract_debounce(distances, 2))
    all_symbolic_properties.append(long_following_abstract_debounce(distances, 3))

all_symbolic_properties.append(veh_in_adj_right_lane_AP())
all_symbolic_properties.append(veh_straddling_ego_lane_AP())
all_symbolic_properties.append(lead_veh_too_close_AP())
all_symbolic_properties.append(cut_in_from_right())
