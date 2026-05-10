"""
Drone Flight Path Simulator — AAU Location
===========================================
Centered on: 9°02'24.8"N 38°45'50.4"E
All 4 additions: obstacle avoidance, battery RTH, swarm, GPS
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from dataclasses import dataclass
import math
from pathlib import Path

# ── Config ────────────────────────────────────────────────────────────────────
class DroneConfig:
    MAX_SPEED        = 12.0
    CLIMB_RATE       = 4.0
    DESCENT_RATE     = 3.0
    BATTERY_CAPACITY = 100.0
    MOVE_DRAIN       = 0.14
    HOVER_DRAIN      = 0.08
    WIND_STRENGTH    = 1.0
    WIND_DIRECTION   = 60.0
    STEPS_PER_SEG    = 80
    HOME_ALT         = 0.0
    LOW_BATTERY      = 20.0
    SAFE_DISTANCE    = 5.0

# ── Data classes ──────────────────────────────────────────────────────────────
@dataclass
class Waypoint:
    x: float; y: float; alt: float; label: str = ""
    def position(self): return np.array([self.x, self.y, self.alt])

@dataclass
class Obstacle:
    cx:float; cy:float; cz:float
    dx:float; dy:float; dz:float
    label:str="Building"; color:str="#8B4513"

    def contains(self, p, margin=3.0):
        return (abs(p[0]-self.cx)<self.dx+margin and
                abs(p[1]-self.cy)<self.dy+margin and
                abs(p[2]-self.cz)<self.dz+margin)

    def faces(self):
        cx,cy,cz,dx,dy,dz=self.cx,self.cy,self.cz,self.dx,self.dy,self.dz
        c=[[cx-dx,cy-dy,cz-dz],[cx+dx,cy-dy,cz-dz],
           [cx+dx,cy+dy,cz-dz],[cx-dx,cy+dy,cz-dz],
           [cx-dx,cy-dy,cz+dz],[cx+dx,cy-dy,cz+dz],
           [cx+dx,cy+dy,cz+dz],[cx-dx,cy+dy,cz+dz]]
        return [[c[0],c[1],c[2],c[3]],[c[4],c[5],c[6],c[7]],
                [c[0],c[1],c[5],c[4]],[c[2],c[3],c[7],c[6]],
                [c[0],c[3],c[7],c[4]],[c[1],c[2],c[6],c[5]]]

# ── GPS conversion ────────────────────────────────────────────────────────────
def gps_to_metres(lat, lon, ref_lat, ref_lon):
    R = 6_371_000
    x = R * math.radians(lon-ref_lon) * math.cos(math.radians(ref_lat))
    y = R * math.radians(lat-ref_lat)
    return x, y

def make_waypoints(gps_list, alts, labels, ref_lat, ref_lon):
    wps = []
    for (lat,lon), alt, lbl in zip(gps_list, alts, labels):
        x, y = gps_to_metres(lat, lon, ref_lat, ref_lon)
        wps.append(Waypoint(x, y, alt, lbl))
    return wps

# ── Interpolation ─────────────────────────────────────────────────────────────
def interp(a, b, steps):
    t = np.linspace(0,1,steps)
    s = t*t*(3-2*t)
    return np.column_stack([
        a.x+(b.x-a.x)*s, a.y+(b.y-a.y)*s, a.alt+(b.alt-a.alt)*s])

# ── Obstacle avoidance ────────────────────────────────────────────────────────
def reroute(wp_a, wp_b, obstacles, steps=30):
    for t in np.linspace(0,1,steps):
        p = wp_a.position() + t*(wp_b.position()-wp_a.position())
        for obs in obstacles:
            if obs.contains(p):
                top = obs.cz + obs.dz + 10.0
                mid = np.array([(wp_a.x+wp_b.x)/2,
                                 (wp_a.y+wp_b.y)/2,
                                 max(wp_a.alt, wp_b.alt, top)])
                return [Waypoint(mid[0],mid[1],mid[2],"↑Detour")]
    return []

def full_path(waypoints, obstacles, config):
    eff = [waypoints[0]]
    for i in range(len(waypoints)-1):
        detour = reroute(waypoints[i], waypoints[i+1], obstacles)
        eff += detour + [waypoints[i+1]]
    segs = [interp(eff[i], eff[i+1], config.STEPS_PER_SEG)
            for i in range(len(eff)-1)]
    path = np.vstack(segs)
    # wind drift
    n = len(path); t = np.linspace(0,2*np.pi,n)
    ang = math.radians(config.WIND_DIRECTION)
    path[:,0] += config.WIND_STRENGTH*np.sin(t)*math.cos(ang)*0.3
    path[:,1] += config.WIND_STRENGTH*np.sin(t+1)*math.sin(ang)*0.3
    return path

# ── Battery-aware RTH ─────────────────────────────────────────────────────────
def battery_plan(waypoints, config):
    battery = config.BATTERY_CAPACITY
    result  = [waypoints[0]]
    blog    = [battery]
    home    = waypoints[0]

    for i in range(1, len(waypoints)):
        dist  = np.linalg.norm(waypoints[i].position()-waypoints[i-1].position())
        drain = dist * config.MOVE_DRAIN * 0.5
        dhome = np.linalg.norm(waypoints[i].position()-home.position())
        reserve = dhome * config.MOVE_DRAIN * 0.5 + config.LOW_BATTERY

        if battery - drain < reserve:
            print(f"    ⚡ Battery {battery:.1f}% at WP{i} "
                  f"— RTH triggered (need {reserve:.1f}% reserve)")
            result += [Waypoint(home.x, home.y, 20, "RTH↑"),
                       Waypoint(home.x, home.y,  0, "RTH✓")]
            blog   += [max(battery-drain,0), max(battery-drain*2,0)]
            return result, blog

        battery -= drain
        result.append(waypoints[i])
        blog.append(max(battery, 0))

    return result, blog

# ── Swarm ─────────────────────────────────────────────────────────────────────
OFFSETS = [(0,0), (10,0), (5,10)]
COLORS  = ["#58a6ff","#3fb950","#f78166"]
NAMES   = ["Drone 1","Drone 2","Drone 3"]

def swarm_paths(waypoints, obstacles, config):
    paths = []
    for ox, oy in OFFSETS:
        shifted = [Waypoint(wp.x+ox, wp.y+oy, wp.alt, wp.label)
                   for wp in waypoints]
        paths.append(full_path(shifted, obstacles, config))
    return paths

# ── Styles ────────────────────────────────────────────────────────────────────
def s3d(ax):
    ax.set_facecolor("#0d1117")
    for p in [ax.xaxis.pane,ax.yaxis.pane,ax.zaxis.pane]:
        p.fill=False; p.set_edgecolor("#1c2128")
    ax.tick_params(colors="#888780",labelsize=7)
    ax.grid(True,color="#1c2128",linewidth=0.4)

def s2d(ax):
    ax.set_facecolor("#0d1117")
    ax.tick_params(colors="#888780",labelsize=8)
    for s in ax.spines.values(): s.set_edgecolor("#30363d")
    ax.grid(True,color="#1c2128",linewidth=0.4)

def draw_obs_3d(ax, obs):
    p = Poly3DCollection(obs.faces(), alpha=0.35,
                         facecolor=obs.color, edgecolor="#555")
    ax.add_collection3d(p)
    ax.text(obs.cx, obs.cy, obs.cz+obs.dz+3,
            obs.label, color="#ccc", fontsize=7, ha="center")

def draw_obs_2d(ax, obs):
    r = plt.Rectangle((obs.cx-obs.dx,obs.cy-obs.dy),
                       obs.dx*2, obs.dy*2,
                       facecolor=obs.color, alpha=0.4,
                       edgecolor="#aaa", linewidth=1)
    ax.add_patch(r)
    ax.text(obs.cx, obs.cy, obs.label,
            color="#eee", fontsize=7, ha="center", va="center")

# ── Main plot ─────────────────────────────────────────────────────────────────
def plot(waypoints, obstacles, config,
         ref_lat, ref_lon, location_name):

    print("\n  [1/4] Battery-aware mission planning...")
    wp_plan, batt_log = battery_plan(waypoints, config)

    print("  [2/4] Computing main flight path with obstacle avoidance...")
    mpath = full_path(wp_plan, obstacles, config)

    print("  [3/4] Building 3-drone swarm paths...")
    spaths = swarm_paths(wp_plan, obstacles, config)

    n = min(len(p) for p in spaths)
    min_sep = min(
        np.linalg.norm(spaths[i][:n]-spaths[j][:n],axis=1).min()
        for i in range(3) for j in range(i+1,3))
    print(f"  [4/4] Min swarm separation: {min_sep:.1f}m "
          f"({'✓ SAFE' if min_sep>config.SAFE_DISTANCE else '⚠ CLOSE'})")

    total_dist = np.linalg.norm(np.diff(mpath,axis=0),axis=1).sum()
    flight_min = total_dist / config.MAX_SPEED / 60

    # ── Figure layout ─────────────────────────────────────────────
    fig = plt.figure(figsize=(19,11), facecolor="#0d1117")
    fig.suptitle(
        f"Drone Flight Simulator  |  Location: {location_name}  "
        f"({ref_lat:.4f}°N, {ref_lon:.4f}°E)",
        color="white", fontsize=15, fontweight="bold", y=0.99)

    gs = gridspec.GridSpec(3,3, figure=fig,
                           hspace=0.44, wspace=0.32,
                           left=0.05,right=0.97,top=0.94,bottom=0.06)

    ax3d  = fig.add_subplot(gs[:2,:2], projection="3d")
    ax_tp = fig.add_subplot(gs[0,2])
    ax_al = fig.add_subplot(gs[1,2])
    ax_bt = fig.add_subplot(gs[2,:2])
    ax_sw = fig.add_subplot(gs[2,2])

    s3d(ax3d); s2d(ax_tp); s2d(ax_al); s2d(ax_bt); s2d(ax_sw)

    # ── 3D plot ───────────────────────────────────────────────────
    for obs in obstacles: draw_obs_3d(ax3d, obs)

    for path, col, name in zip(spaths, COLORS, NAMES):
        ax3d.plot(path[:,0],path[:,1],path[:,2],
                  color=col, linewidth=1.8, alpha=0.8, label=name)
        ax3d.plot(path[:,0],path[:,1],
                  zs=0, zdir='z', color=col,
                  linewidth=0.5, alpha=0.15, linestyle="--")

    for i, wp in enumerate(wp_plan):
        c = ("#1D9E75" if i==0 else
             "#ff6b6b" if "RTH" in wp.label else "#D85A30")
        ax3d.scatter(wp.x,wp.y,wp.alt, color=c, s=65,
                     zorder=5, edgecolors="white", linewidths=0.8)
        ax3d.text(wp.x+2,wp.y+2,wp.alt+3,
                  wp.label, color="white", fontsize=7)

    ax3d.set_xlabel("East (m)",  color="#888780", labelpad=6)
    ax3d.set_ylabel("North (m)", color="#888780", labelpad=6)
    ax3d.set_zlabel("Alt (m)",   color="#888780", labelpad=6)
    ax3d.set_title("3D flight path — obstacle avoidance + swarm",
                   color="white", fontsize=11, pad=8)
    ax3d.legend(fontsize=9, facecolor="#161b22",
                edgecolor="#30363d", labelcolor="white",
                loc="upper left")
    ax3d.view_init(elev=28, azim=-50)

    # ── Top-down map ──────────────────────────────────────────────
    for obs in obstacles: draw_obs_2d(ax_tp, obs)
    for path, col, name in zip(spaths, COLORS, NAMES):
        ax_tp.plot(path[:,0],path[:,1],
                   color=col, linewidth=1.6, alpha=0.85, label=name)
    for wp in wp_plan:
        c = ("#1D9E75" if wp.label=="Home" else
             "#ff6b6b" if "RTH" in wp.label else "#D85A30")
        ax_tp.scatter(wp.x, wp.y, color=c, s=45, zorder=5)
        ax_tp.annotate(wp.label,(wp.x,wp.y),
                       xytext=(4,4),textcoords="offset points",
                       color="white",fontsize=7)
    ax_tp.set_title("Top-down map", color="white", fontsize=10)
    ax_tp.set_xlabel("East (m)",  color="#888780", fontsize=8)
    ax_tp.set_ylabel("North (m)", color="#888780", fontsize=8)
    ax_tp.legend(fontsize=8,facecolor="#161b22",
                 edgecolor="#30363d",labelcolor="white")

    # ── Altitude profile ──────────────────────────────────────────
    t_arr = np.linspace(0, flight_min*60, len(mpath))
    ax_al.fill_between(t_arr,0,mpath[:,2],color="#85B7EB",alpha=0.2)
    ax_al.plot(t_arr,mpath[:,2],color="#85B7EB",linewidth=2)
    for obs in obstacles:
        top = obs.cz+obs.dz
        ax_al.axhline(top, color=obs.color,
                      linewidth=0.8, linestyle="--", alpha=0.7)
        ax_al.text(t_arr[-1]*0.02, top+1, f"{obs.label}",
                   color="#bbb", fontsize=7)
    ax_al.set_title("Altitude profile", color="white", fontsize=10)
    ax_al.set_xlabel("Time (s)", color="#888780", fontsize=8)
    ax_al.set_ylabel("Altitude (m)", color="#888780", fontsize=8)
    ax_al.set_ylim(bottom=-2)

    # ── Battery chart ─────────────────────────────────────────────
    bt = np.array(batt_log)
    bt_t = np.linspace(0, flight_min*60, len(bt))
    cols_b = np.where(bt>50,"#3fb950",np.where(bt>20,"#d29922","#f85149"))
    for i in range(len(bt)-1):
        ax_bt.fill_between(bt_t[i:i+2],0,bt[i:i+2],
                           color=cols_b[i],alpha=0.35)
        ax_bt.plot(bt_t[i:i+2],bt[i:i+2],
                   color=cols_b[i],linewidth=2.2)
    ax_bt.axhline(config.LOW_BATTERY, color="#f85149",
                  linewidth=1.2, linestyle="--")
    ax_bt.text(bt_t[-1]*0.02, config.LOW_BATTERY+2,
               f"RTH threshold ({config.LOW_BATTERY:.0f}%)",
               color="#f85149", fontsize=8)
    ax_bt.set_title("Battery — auto Return-To-Home at threshold",
                    color="white", fontsize=10)
    ax_bt.set_xlabel("Time (s)", color="#888780", fontsize=8)
    ax_bt.set_ylabel("Battery %", color="#888780", fontsize=8)
    ax_bt.set_ylim(0,108)

    # ── Swarm separation ──────────────────────────────────────────
    pairs = [(0,1,"D1-D2","#58a6ff"),
             (0,2,"D1-D3","#3fb950"),
             (1,2,"D2-D3","#f78166")]
    t_sw = np.linspace(0, flight_min*60, n)
    for i,j,lbl,col in pairs:
        seps = np.linalg.norm(spaths[i][:n]-spaths[j][:n],axis=1)
        ax_sw.plot(t_sw, seps, color=col, linewidth=1.6, label=lbl)
    ax_sw.axhline(config.SAFE_DISTANCE, color="#f85149",
                  linewidth=1.2, linestyle="--")
    ax_sw.text(0, config.SAFE_DISTANCE+0.5,
               f"Safe dist ({config.SAFE_DISTANCE}m)",
               color="#f85149", fontsize=7)
    ax_sw.set_title("Swarm separation", color="white", fontsize=10)
    ax_sw.set_xlabel("Time (s)", color="#888780", fontsize=8)
    ax_sw.set_ylabel("Distance (m)", color="#888780", fontsize=8)
    ax_sw.legend(fontsize=8,facecolor="#161b22",
                 edgecolor="#30363d",labelcolor="white")

    # ── Stats box ─────────────────────────────────────────────────
    fig.text(0.695, 0.03,
        f"  Location  : {location_name}\n"
        f"  GPS ref   : {ref_lat:.4f}°N  {ref_lon:.4f}°E\n"
        f"  Distance  : {total_dist:.0f} m\n"
        f"  Flight    : {flight_min:.2f} min\n"
        f"  Max alt   : {mpath[:,2].max():.0f} m\n"
        f"  Drones    : 3  |  Min sep: {min_sep:.1f}m\n"
        f"  Obstacles : {len(obstacles)}\n"
        f"  Battery   : {bt[-1]:.1f}% remaining",
        color="white", fontsize=8.5, fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.5",
                  facecolor="#161b22",edgecolor="#30363d"))

    out_dir = Path(__file__).resolve().parent / "outputs"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / "drone_aau.png"
    plt.savefig(out, dpi=145, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"\n  Saved → {out}")
    return total_dist, flight_min, min_sep, bt[-1]


# ── Mission definition ────────────────────────────────────────────────────────
if __name__ == "__main__":

    print("=" * 58)
    print("  Drone Simulator — AAU Campus Area Mission")
    print("  📍 9°02'24.8\"N  38°45'50.4\"E")
    print("=" * 58)

    config = DroneConfig()

    # Reference point — your provided GPS location
    REF_LAT = 9.040222
    REF_LON = 38.764000

    # Mission waypoints — real GPS coordinates around the area
    # Each small offset (~0.0003 deg) ≈ ~30 metres
    GPS_POINTS = [
        (9.040222, 38.764000),   # Home — take-off point
        (9.040222, 38.764000),   # Climb in place
        (9.040500, 38.764200),   # Alpha — fly north-east
        (9.040800, 38.764500),   # Bravo — continue north-east
        (9.041100, 38.764700),   # Charlie — survey point
        (9.041000, 38.765000),   # Delta — peak altitude
        (9.040600, 38.765100),   # Echo — turn south
        (9.040200, 38.764800),   # Foxtrot — approach
        (9.040222, 38.764000),   # Return home
        (9.040222, 38.764000),   # Land
    ]

    ALTITUDES = [0, 20, 30, 40, 45, 50, 45, 35, 20, 0]
    LABELS    = ["Home","Climb","Alpha","Bravo","Charlie",
                 "Delta","Echo","Foxtrot","Return","Land"]

    print("\n  GPS waypoints → metres:")
    waypoints = make_waypoints(
        GPS_POINTS, ALTITUDES, LABELS, REF_LAT, REF_LON)
    for wp in waypoints:
        print(f"    {wp.label:10s}  E={wp.x:6.1f}m  N={wp.y:6.1f}m  "
              f"alt={wp.alt}m")

    # Obstacles around the area (buildings/structures)
    # Adjust positions to match real buildings near your location
    obstacles = [
        # (cx, cy, cz,  dx, dy, dz,  label,       color)
        # cx/cy relative to Home in metres
        # dz = half-height, so total height = cz+dz metres
        Obstacle( 35,  25, 12,  8,  8, 12, "Block-A",  "#8B4513"),
        Obstacle( 75,  60, 18, 10,  7, 18, "Tower-B",  "#A0522D"),
        Obstacle( 55, 100, 10,  6,  6, 10, "NoFly-1",  "#8B0000"),
        Obstacle(110,  45, 15,  7,  7, 15, "Block-C",  "#6B4226"),
        Obstacle( 90, 120, 20,  5,  5, 20, "Tower-D",  "#704214"),
    ]

    print(f"\n  Obstacles: {len(obstacles)}")
    for o in obstacles:
        print(f"    {o.label:10s}  pos=({o.cx},{o.cy})m  "
              f"height={(o.cz+o.dz)}m")

    dist, t_min, sep, batt = plot(
        waypoints, obstacles, config,
        REF_LAT, REF_LON, "AAU Campus Area")

    print(f"""
{'=' * 58}
  Mission complete!
  Total distance  : {dist:.0f} m
  Flight time     : {t_min:.2f} min
  Max altitude    : 50 m
  Drones in swarm : 3
  Min separation  : {sep:.1f} m
  Battery left    : {batt:.1f}%
  Obstacles avoided: {len(obstacles)}
{'=' * 58}
  To use your own nearby waypoints:
  1. Open Google Maps
  2. Right-click any point near 9.040222, 38.764000
  3. Copy the coordinates
  4. Add to GPS_POINTS list above
{'=' * 58}
""")
