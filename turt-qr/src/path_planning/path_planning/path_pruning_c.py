import numpy as np
from path_planning.ellipses2 import Ellipse2

def _euclidean_distance(node1, node2):
        return np.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

def _score_nodes(path: list, map, step_size: int):
    """
    Scores nodes based on obstacle proximity.
    """
    map_data = map.data
    map_width = map.info.width
    map_height = map.info.height
    resolution = map.info.resolution
    origin_x = map.info.origin.position.x
    origin_y = map.info.origin.position.y

    def world_to_grid(x, y):
        col = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)
        return row, col

    def get_cell(row, col):
        if 0 <= row < map_height and 0 <= col < map_width:
            return map_data[row * map_width + col]
        return 0

    radius = max(1, int(step_size))
    scores = []

    for node in path: # for each obstacle nearby add to score
        row, col = world_to_grid(node[0], node[1])
        score = 0
        count = 0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                val = get_cell(row + dr, col + dc)
                if val == 100:
                    score += 1
                count += 1
        scores.append(score / count if count > 0 else 0)

    return scores

def _prune_path(path: list, map) -> list:
    if len(path) <= 2:
        return path

    map_data = map.data
    map_width = map.info.width
    map_height = map.info.height
    resolution = map.info.resolution
    origin_x = map.info.origin.position.x
    origin_y = map.info.origin.position.y

    def world_to_grid(x, y):
        col = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)
        return row, col

    def get_cell(row, col):
        if 0 <= row < map_height and 0 <= col < map_width:
            return map_data[row * map_width + col]
        return 100  # treat out of bounds as occupied

    def line_of_sight(p1, p2):
        """
        Line of sight pruning.
        """
        x1, y1 = p1
        x2, y2 = p2
        dist = np.hypot(x2 - x1, y2 - y1)
        steps = max(int(dist / (resolution * 0.5)), 1)
        for k in range(steps + 1):
            t = k / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            row, col = world_to_grid(x, y)
            if get_cell(row, col) == 100:
                return False
        return True

    pruned_path = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(path[i], path[j]):
                break
            j -= 1
        pruned_path.append(path[j])
        i = j

    return pruned_path

def _check_qr_string_length(path: list, f1_idx: int, f2_idx: int, char_limit: int=25):
    """
    Returns (qr_len, qr_data_list) by building the exact on-wire payload
    for the given f1/f2 indices.

    Parameters
    ----------
    path : list
        List of points from RRT#.
    f1_idx : int
        Index of first ellipse focal point.
    f2_idx : int
        Index of second ellipse focal point.
    char_limit : int
        Character limit of QR-code.

    Returns
    -------
    qr_len : int
        Length of Qr-code
    qr_data_list : list
        List of points in Qr-code. Example [x, y, s]
    """
    # Compute the ellipse parameter s from path length between f1 and f2
    path_wo_goal = path[:-1]
    f1_idx = max(0, min(f1_idx, len(path_wo_goal) - 1))
    f2_idx = max(0, min(f2_idx, len(path_wo_goal) - 1))
    if f1_idx > f2_idx:
        f1_idx, f2_idx = f2_idx, f1_idx

    L_sum = 0.0
    for i in range(f1_idx, f2_idx):
        p1, p2 = path_wo_goal[i], path_wo_goal[i + 1]
        L_sum += _euclidean_distance(p1, p2)

    # use Ellipse2 to get the same 's' you encode elsewhere
    temp_ellipse = Ellipse2(path_wo_goal[f1_idx], path_wo_goal[f2_idx], L_sum)
    s_enc = np.ceil(temp_ellipse.s)

    qr_data, qr_len, _ = _build_qr_for_indices(path, f1_idx, f2_idx, s_enc, char_limit)
    if qr_data is None:
        return char_limit + 1, []
    return qr_len, qr_data

def _build_qr_for_indices(path, f1_idx, f2_idx, s_value, char_limit=25):
    """
    Build the QR payload per spec and return (qr_data_list, qr_len, path_out).

    Parameters
    ----------
    path : list
        List of points from RRT#.
    f1_idx : int
        Index of first ellipse focal point.
    f2_idx : int
        Index of second ellipse focal point.
    s_value : int
        Distance between ellipse foci.
    char_limit : int
        Character limit of QR-code.

    Encoding rules
    --------------
      - For nodes BEFORE f1: (x, y, 0)
      - At f1: (x, y, s_value)
      - Immediately AFTER f1: f2 as (x, y) pair unless f2 == goal (then omit)
      - For nodes AFTER f2 (excluding goal): (x, y, 0)
      - Goal is NEVER encoded and appended back on decode.
      - The string may end with (x,y) or (x,y,s), but never a single x.
    
    Returns
    -------
    qr : list
        List of points in qr code. Example [x y s].
    qr_len : int
        Length of Qr str
    output_path : list
        Pruned path with ellipse. 
    """
    path_wo_goal = path[:-1]
    N = len(path_wo_goal)
    # Clamp & order
    f1_idx = max(0, min(f1_idx, N - 1))
    f2_idx = max(0, min(f2_idx, N - 1))
    if f1_idx > f2_idx:
        f1_idx, f2_idx = f2_idx, f1_idx

    def would_fit(cur, add):
        return len(' '.join(map(str, cur + add))) <= char_limit

    qr = []

    # 1) pre-f1 triples
    for i in range(0, f1_idx):
        triple = [path_wo_goal[i][0], path_wo_goal[i][1], 0]
        if not would_fit(qr, triple):
            return None, char_limit + 1, None
        qr.extend(triple)

    # 2) f1 triple
    f1 = path_wo_goal[f1_idx]
    triple_f1 = [f1[0], f1[1], int(round(s_value))]
    if not would_fit(qr, triple_f1):
        return None, char_limit + 1, None
    qr.extend(triple_f1)

    # 3) f2 pair unless f2 == goal
    goal = path[-1]
    f2 = path_wo_goal[f2_idx]
    if f2 != goal:
        pair_f2 = [f2[0], f2[1]]
        if not would_fit(qr, pair_f2):
            return None, char_limit + 1, None
        qr.extend(pair_f2)

    # 4) post-f2 triples
    for i in range(f2_idx + 1, N):
        triple = [path_wo_goal[i][0], path_wo_goal[i][1], 0]
        if not would_fit(qr, triple):
            return None, char_limit + 1, None
        qr.extend(triple)

    # Output path that is “represented”: [.. up to f1] + [f2 .. end] + goal
    path_out = path_wo_goal[:f1_idx + 1] + path_wo_goal[f2_idx:] + [path[-1]]

    return qr, len(' '.join(map(str, qr))), path_out

def _greedy_ell(path, scores, char_limit=25):
    """
    Greedy algorithm to select ellipse foci based on node scores & QR limit.
    Handles incomplete/short paths safely.

    Parameters
    ----------
    path : list
        List of points from RRT#
    scores : list
        List of scores for each point in path.
    char_limit : int
        Character limit of Qr-code.

    Returns
    -------
    Dict with keys
    {'f1_idx'
        'f2_idx',
        'f1',
        'f2',
        'a',
        'b', 
        's',
        'tilt',
        'center',
        'eccentricity',
        'area',
        'path'}
    """
    if not path or not scores:
        return None

    path_wo_goal = path[:-1]
    N = len(path_wo_goal)

    # Need at least 2 points (start+something) besides goal
    if N < 2:
        return None

    # Initial guess: local max and neighbor
    max_index = int(np.argmax(scores[:N]))  # only consider non-goal
    neighbors = [i for i in (max_index - 1, max_index + 1) if 0 <= i < N]
    if not neighbors:
        return None
    best_neighbor = max(neighbors, key=lambda i: scores[i])
    f1_idx, f2_idx = sorted([max_index, best_neighbor])

    # Expand until fits
    while True:
        # Guard: if indices invalid, stop
        if f1_idx < 0 or f2_idx >= N or f1_idx >= f2_idx:
            return None

        qr_len, _ = _check_qr_string_length(path, f1_idx, f2_idx, char_limit)
        if qr_len <= char_limit:
            break

        candidates = []
        if f1_idx - 1 >= 0:
            candidates.append(f1_idx - 1)
        if f2_idx + 1 < N:
            candidates.append(f2_idx + 1)

        if not candidates:
            break

        best = max(candidates, key=lambda idx: scores[idx])
        if best < f1_idx:
            f1_idx = best
        else:
            f2_idx = best

    # --- Final guard ---
    if f1_idx < 0 or f2_idx >= N or f1_idx >= f2_idx:
        return None

    # Compute path length safely
    L_sum = 0.0
    for i in range(f1_idx, f2_idx):
        if i + 1 < N:
            p1, p2 = path_wo_goal[i], path_wo_goal[i + 1]
            L_sum += _euclidean_distance(p1, p2)

    f1, f2 = path_wo_goal[f1_idx], path_wo_goal[f2_idx]
    ellipse = Ellipse2(f1, f2, L_sum)
    s_enc = np.ceil(ellipse.s)

    return {
        'f1_idx': f1_idx,
        'f2_idx': f2_idx,
        'f1': f1,
        'f2': f2,
        'a': ellipse.a,
        'b': np.ceil(ellipse.b),
        's': s_enc,
        'tilt': ellipse.tilt,
        'center': (ellipse.h, ellipse.k),
        'eccentricity': ellipse.e,
        'area': ellipse.area,
        'path': path_wo_goal[:f1_idx+1] + path_wo_goal[f2_idx:] + [path[-1]]
    }

def fit_to_qr(path: list, map_obj: list, e, step_size, char_limit: int=25):
    """
    Takes path from RRT# and completes pruning pipeline.

    Parameters
    ----------
    path : list
        List of points from RRT#.
    map_obj : nav_msgs.msg.OccupancyGrid
        ROS map object.
    char_limit : int
        Character limit for QR-code.
    
    Returns
    -------
    qr : str
        Path string that fits in character limit.
    """
    # make sure int to fit in qr
    path = [tuple(round(x) for x in t) for t in path]

    # Initial encoding
    print(f"orignal path: {path}\n")
    line_of_sight_path = _prune_path(path, map_obj)
    print(f"Line of sight path: {line_of_sight_path}\n")
    path_scores = _score_nodes(path, map_obj, step_size)

    # check if fits
    # Exclude the goal node for QR string generation
    short_path_wo_goal = line_of_sight_path[:-1]
    
    # Identify how many pts need to be cut
    test = [round(x) for item in short_path_wo_goal for x in (item if isinstance(item, tuple) else (item,))]
    test_with_zeros = []
    for i in range(len(test)-1):
        test_with_zeros.append(test[i])
        if i % 2 == 1 and i != len(test) - 1: 
            test_with_zeros.append(0)
    data_str = ' '.join(map(str, test_with_zeros))
    curr_ct = len(data_str)

    if curr_ct > char_limit:
        params = _greedy_ell(line_of_sight_path, path_scores, char_limit)

        qr_data, qr_len, path_out = _build_qr_for_indices(line_of_sight_path, 
                                                          params['f1_idx'], 
                                                          params['f2_idx'], 
                                                          params['s'], 
                                                          char_limit)

        qr_str = ' '.join(map(str, qr_data))
    elif curr_ct <= char_limit:
        final_path = short_path_wo_goal
        qr_data = []
        for pt in final_path:
            tuple_to_add = [pt[0], pt[1], 0]
            temp_str = ' '.join(map(str, qr_data + tuple_to_add))
            if len(temp_str) > char_limit:
                break
            qr_data.extend(tuple_to_add)
        qr_str = ' '.join(map(str, qr_data))
    
    print(f"Path with ell: {qr_data}")
    print(f'Final QR string: {qr_str} with char ct. {len(qr_str)}\n')
    return qr_str

def decode_qr_string(qr_string, goal):
    """
    Decode the QR payload created by _build_qr_for_indices:
      - Triples (x,y,s) everywhere EXCEPT for a single f2 pair immediately after f1's triple
        (unless f2 == goal, then that pair is omitted).
      - Goal is never encoded; always appended.
      - The string ends with either (x,y) or (x,y,s) but never a single x.
    """
    from ellipses2 import Ellipse2

    data = list(map(int, qr_string.split()))
    path = []
    ellipse = None
    f1 = f2 = None
    s_val = None

    i = 0
    while i < len(data):
        # If we have at least a triple available, try to read (x,y,s)
        if i + 2 < len(data):
            x, y, s = data[i], data[i + 1], data[i + 2]
            if s != 0 and ellipse is None:
                # This is f1
                f1 = (x, y)
                s_val = s
                path.append((x, y))

                # Expect f2 pair immediately after f1 triple (if present)
                if i + 4 < len(data):
                    f2 = (data[i + 3], data[i + 4])
                    path.append(f2)
                    i += 5  # consumed triple + pair
                else:
                    # No pair after f1: f2 must be the known goal
                    f2 = goal
                    i += 3  # consumed triple
                continue
            else:
                # Regular triple (x,y,0)
                path.append((x, y))
                i += 3
                continue
        # If only 2 numbers remain, treat them as a plain (x,y) pair (e.g., f2 at end)
        elif i + 1 < len(data):
            path.append((data[i], data[i + 1]))
            i += 2
            continue
        else:
            break

    # Append the known goal at the end
    path.append(goal)

    if f1 is not None and f2 is not None:
        ellipse = Ellipse2(f1, f2, s_val)

    return path, ellipse

if __name__=="__main__":

    qr_string = "45 45 0 46 34 49 12 2"
    goal = (5, 5)

    path, ellipse = decode_qr_string(qr_string, goal)
    print(f"Path: {path}\n")
    print(f"Ellipse f1: {ellipse.f1} | Ellipse f2:{ellipse.f2}\n")

    