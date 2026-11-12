# Risk-Aware Convoy Routing

## Done (Week 1)
- `app.html`: map interface (Leaflet + OSRM + Nominatim + Open-Meteo), Tirana ambulance test case
- Synthetic grid road network with travel time + risk per road
- Risk-weighted A*: cost = time * (1 + alpha * risk), closed roads = impassable
- Live events (road closed / risk changed) applied while the vehicle drives
- Replanning agent vs. fixed shortest-path baseline
- 50-scenario experiment (`python convoy.py`)

## Done (Week 2)
- Animated ambulance marker moving along the current route
- Automatic reroute when new blockades appear while the ambulance is in motion
- Reused the same risk-aware planner to compare the live route against the normal baseline
- Kept the app as a single HTML file with no build step

## Done (Week 3)
- Added `blockades.json` with approximate central Tirana locations that can be edited from local news reports and OpenStreetMap
- Added a “Load blockades from file” button to load the JSON data into the map
- Added a “Use nearest real hospital” button that searches Overpass for hospitals near the patient and sets the ambulance start point to the closest one
- Added a time slider so blockades are active only during their chosen time window
- Kept the app as a single-file browser app with no build step

## Result so far (50 scenarios, alpha=3)
| | success | avg time | avg risk |
|---|---|---|---|
| baseline | 24% | 22.6 | 3.32 |
| agent | 100% | 24.2 | 0.19 |

(time/risk averaged over successful runs only)

## Next steps
1. Week 3: plots (route before/after replanning, time vs risk trade-off), sweep alpha
2. Week 4: multiple delivery stops for one vehicle (nearest-neighbour / TSP order)
3. Week 5: text updates -> road changes with a local LLM ("bridge at X collapsed")
4. Week 6: more baselines (replan-shortest-only), 2+ vehicles
5. Final: report, limitations, demo

## Ideas to improve realism
- Load a real map (OSMnx) instead of a grid
- Risk that spreads/decays over time
- Penalty for waiting vs. detouring when a road is closed

## What changed this week
- The map can now show the ambulance moving along the best route instead of only drawing the path.
- When a blockade is added or moved while the vehicle is en route, the planner immediately tries a new route from the current vehicle position.
- A JSON blockade file can now be loaded from the app, so the user can edit approximate central Tirana locations without changing the JavaScript.
- The app can also fetch the nearest real hospital near the patient through Overpass and route from that hospital as the new start point.
- The time slider lets each blockade act like a temporary event: active only during its chosen minute range, and faded/ignored when inactive.
- This keeps the project focused on a small but visible improvement: a live emergency response demo that is easy to explain in class.

## Next ideas for later weeks
1. Animate the ambulance driving along the route in real time.
2. Add a text box where a typed update like “protest on Rruga e Kavajes” becomes a new blockade automatically.
3. Compare the replanning agent against the plain shortest-path baseline across many scenarios with charts.
4. Extend the system to multiple stops or multiple vehicles.
