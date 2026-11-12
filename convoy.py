"""
Risk-Aware Convoy Routing  --  Week 1 prototype
------------------------------------------------
Single vehicle, synthetic grid road network, live risk/closure events.

Compares:
  * BASELINE : shortest path computed once, never replanned
  * AGENT    : risk-weighted A*, replans whenever an update arrives

Run:  python convoy.py
"""
import copy
import heapq
import math
import random
from dataclasses import dataclass


# ---------------------------------------------------------------- network
@dataclass
class Road:
    time: float            # travel time in minutes
    risk: float = 0.0      # 0 (safe) .. 1 (very dangerous)
    closed: bool = False


class RoadNetwork:
    def __init__(self):
        self.pos = {}      # node -> (x, y)
        self.adj = {}      # node -> {neighbor: Road}  (Road object shared both ways)

    def add_node(self, n, x, y):
        self.pos[n] = (x, y)
        self.adj.setdefault(n, {})

    def add_road(self, u, v, time, risk=0.0):
        road = Road(time, risk)
        self.adj[u][v] = road
        self.adj[v][u] = road

    def road(self, u, v):
        return self.adj[u][v]


def grid_network(rows=10, cols=10, seed=0, hot_zones=3):
    """Grid of towns. Edge times vary; a few 'hot zones' start out risky."""
    rng = random.Random(seed)
    net = RoadNetwork()
    for r in range(rows):
        for c in range(cols):
            net.add_node((r, c), c, r)
    for r in range(rows):
        for c in range(cols):
            for dr, dc in ((0, 1), (1, 0)):
                nr, nc = r + dr, c + dc
                if nr < rows and nc < cols:
                    net.add_road((r, c), (nr, nc), time=rng.uniform(1.0, 2.0))
    # hot zones: circular areas with elevated risk
    for _ in range(hot_zones):
        cx, cy = rng.uniform(0, cols - 1), rng.uniform(0, rows - 1)
        rad, peak = rng.uniform(1.5, 2.5), rng.uniform(0.5, 0.9)
        for u, nbrs in net.adj.items():
            for v, road in nbrs.items():
                mx = (net.pos[u][0] + net.pos[v][0]) / 2
                my = (net.pos[u][1] + net.pos[v][1]) / 2
                d = math.hypot(mx - cx, my - cy)
                if d < rad:
                    road.risk = max(road.risk, peak * (1 - d / rad))
    return net


# ---------------------------------------------------------------- planning
MIN_TIME_PER_UNIT = 1.0   # fastest any road can be -> keeps heuristic admissible


def edge_cost(road, alpha):
    """Risk-weighted cost. alpha=0 gives plain travel time."""
    if road.closed:
        return math.inf
    return road.time * (1 + alpha * road.risk)


def astar(net, start, goal, alpha=0.0):
    """Risk-weighted A*. Returns (path, cost) or (None, inf)."""
    def h(n):
        (x1, y1), (x2, y2) = net.pos[n], net.pos[goal]
        return math.hypot(x1 - x2, y1 - y2) * MIN_TIME_PER_UNIT

    open_heap = [(h(start), 0.0, start)]
    g = {start: 0.0}
    parent = {}
    while open_heap:
        _, gc, u = heapq.heappop(open_heap)
        if u == goal:
            path = [u]
            while u in parent:
                u = parent[u]
                path.append(u)
            return path[::-1], gc
        if gc > g.get(u, math.inf):
            continue
        for v, road in net.adj[u].items():
            c = edge_cost(road, alpha)
            if c == math.inf:
                continue
            ng = gc + c
            if ng < g.get(v, math.inf):
                g[v] = ng
                parent[v] = u
                heapq.heappush(open_heap, (ng + h(v), ng, v))
    return None, math.inf


# ---------------------------------------------------------------- events
@dataclass
class Event:
    time: float
    u: tuple
    v: tuple
    risk: float | None = None     # new risk value, or None
    closed: bool | None = None    # new closed state, or None


def apply_event(net, ev):
    road = net.road(ev.u, ev.v)
    if ev.risk is not None:
        road.risk = ev.risk
    if ev.closed is not None:
        road.closed = ev.closed


# ---------------------------------------------------------------- simulation
@dataclass
class Result:
    arrived: bool
    time: float
    risk: float        # sum of (risk * time) over traversed roads
    replans: int


def simulate(net0, events, start, goal, alpha, replan):
    """Drive one vehicle. If replan=False the route is fixed (baseline)."""
    net = copy.deepcopy(net0)
    events = sorted(events, key=lambda e: e.time)
    path, _ = astar(net, start, goal, alpha)
    if path is None:
        return Result(False, 0, 0, 0)
    t = total_risk = 0.0
    replans = 0
    i = 0          # index in path
    ei = 0         # next event
    while path[i] != goal:
        changed = False
        while ei < len(events) and events[ei].time <= t:
            apply_event(net, events[ei]); ei += 1; changed = True
        u, v = path[i], path[i + 1]
        if replan and changed:
            new_path, _ = astar(net, u, goal, alpha)
            if new_path is None:
                return Result(False, t, total_risk, replans)
            if new_path != path[i:]:
                path, i = new_path, 0
                replans += 1
                u, v = path[0], path[1]
        road = net.road(u, v)
        if road.closed:                       # fixed route hits a closure
            return Result(False, t, total_risk, replans)
        t += road.time
        total_risk += road.risk * road.time
        i += 1
    return Result(True, t, total_risk, replans)


# ---------------------------------------------------------------- scenarios
def make_scenario(seed, rows=10, cols=10, n_events=8):
    rng = random.Random(seed)
    net = grid_network(rows, cols, seed)
    start, goal = (0, 0), (rows - 1, cols - 1)
    base_path, _ = astar(net, start, goal, 0.0)
    events = []
    for _ in range(n_events):
        # half the events hit the baseline route, half are random roads
        if rng.random() < 0.5:
            k = rng.randrange(len(base_path) - 1)
            u, v = base_path[k], base_path[k + 1]
        else:
            u = (rng.randrange(rows), rng.randrange(cols))
            nbrs = list(net.adj[u])
            v = rng.choice(nbrs)
        t = rng.uniform(0, 12)
        if rng.random() < 0.4:
            events.append(Event(t, u, v, closed=True))
        else:
            events.append(Event(t, u, v, risk=rng.uniform(0.6, 1.0)))
    return net, events, start, goal


def run_experiment(n=50, alpha=3.0):
    rows = []
    for s in range(n):
        net, ev, a, b = make_scenario(s)
        base = simulate(net, ev, a, b, alpha=0.0, replan=False)
        agent = simulate(net, ev, a, b, alpha=alpha, replan=True)
        rows.append((base, agent))
    return rows


def report(rows):
    def stats(results):
        ok = [r for r in results if r.arrived]
        n = len(results)
        return (len(ok) / n,
                sum(r.time for r in ok) / max(len(ok), 1),
                sum(r.risk for r in ok) / max(len(ok), 1))
    print(f"{'':10}{'success':>9}{'avg time':>10}{'avg risk':>10}")
    for name, idx in (("baseline", 0), ("agent", 1)):
        s, t, r = stats([x[idx] for x in rows])
        print(f"{name:10}{s:>9.0%}{t:>10.1f}{r:>10.2f}")


if __name__ == "__main__":
    report(run_experiment(n=50, alpha=3.0))
