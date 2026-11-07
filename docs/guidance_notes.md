# Guidance notes

Notes on how landing-guidance-sim works and what the experiments showed. Every number below was produced by this repo with the commands shown, using the default Moon scenario unless stated otherwise.

Default scenario: gravity 1.62 m/s^2, dry mass 1000 kg, fuel 300 kg, max thrust 6000 N, start 500 m up, 50 m sideways of the pad, descending at 40 m/s.

## 1. Two layers

The simulator separates two jobs:

- **Guidance** decides what the lander should be doing: given altitude and velocity, what vertical velocity do we want right now?
- **Control** makes it happen: given the wanted velocity, how much throttle and tilt should the engine use?

Keeping them apart means a guidance law can be swapped without touching the controller, which is what makes the comparisons below possible.

## 2. Physics

The lander is a point mass in 2D with an engine that can tilt by an angle `theta` from vertical.

```
ax = (T / m) * sin(theta)
ay = (T / m) * cos(theta) - g
m  = dry_mass + fuel
dm/dt = -mdot_max * throttle
```

The state is updated with semi-implicit Euler: velocity first, then position using the new velocity. The engine is either off or between `min_throttle` and full throttle, and `theta` is clamped to `max_tilt_deg`.

## 3. Guidance laws

Both laws output a desired vertical velocity `v_des(h)` for the current altitude `h`.

### Constant deceleration

If the lander brakes at a steady deceleration `a` and should reach the ground at touchdown speed `v_td`, then from `v^2 = v_td^2 + 2 a h`:

```
v_des(h) = -sqrt(v_td^2 + 2 * a * h)
```

`a` is the setting `const_decel` (default 1.0 m/s^2). The curve is fixed in advance and does not depend on the lander's mass or thrust.

### Late burn (suicide burn)

Same curve, but `a` is the largest braking the engine can actually deliver, with a safety factor:

```
a = burn_safety * (max_thrust / m - g)
```

It is recomputed every step because `m` falls as fuel burns. While the lander is slower than the curve (`vy` above `v_des`), no braking is needed, so the law returns a target far below the current velocity. The controller then keeps the engine off. Once the lander reaches the curve, the engine comes on and follows it down.

## 4. Controller

**Vertical.** A PID acts on the velocity error `v_des - vy`. Its output plus gravity feedforward gives the vertical acceleration to produce:

```
lift   = g + PID_vertical(v_des - vy)
thrust = m * lift / cos(theta)
```

Thrust below half of the minimum throttle means the engine turns off, and anything else is clamped into the engine's range. The PID has an anti-windup clamp on its integral term, and both PIDs are reset while the engine is off, so a long coast does not leave stored error behind.

**Horizontal.** The lander steers toward the pad in two stages:

```
desired_vx = clamp(-kp_pos * x, -max_vx_cmd, +max_vx_cmd)
ax_cmd     = PID_horizontal(desired_vx - vx)
theta      = atan2(ax_cmd, lift), clamped to +/- max_tilt
```

## 5. Success criteria

Touchdown is found by linear interpolation between the last two steps, so the speeds are those at the moment of ground contact and not those of the step after it. A landing counts as a success when all of these hold (a value exactly on a limit passes):

| Check | Limit |
| --- | --- |
| Vertical speed | at most 2 m/s |
| Sideways speed | at most 1 m/s |
| Distance from pad | at most 5 m |

## 6. Findings

### 6.1 Late burn is faster and cheaper, constant deceleration is gentler

```powershell
./build/lander.exe --run "guidance=constant-decel" --run "guidance=late-burn"
```

| Moon | Time | Landing speed | Fuel used |
| --- | --- | --- | --- |
| Constant decel | 31.09 s | 0.85 m/s | 38.82 kg |
| Late burn | 18.43 s | 1.71 m/s | 29.69 kg |

Late burn saves about 9.1 kg of fuel and 12.7 s. The same comparison on Mars settings (`--config configs/mars.cfg`) gives 65.48 kg versus 44.33 kg, and 30.98 s versus 17.53 s.

A reasonable explanation, not tested separately: constant deceleration keeps the engine running for most of the descent, so a large part of the fuel goes into holding the lander up against gravity. Late burn spends most of the time falling for free and burns hard only at the end.

The price is margin. Late burn lands at 1.71 m/s against the 2.0 m/s limit, and on Mars at 1.91 m/s. Constant decel lands at 0.85 m/s on the Moon and 0.88 m/s on Mars.

![Guidance comparison](screenshots/guidance_comparison.png)

### 6.2 Late burn cannot steer from far off

```powershell
./build/lander.exe --run "guidance=late-burn init_x=100" --run "guidance=constant-decel init_x=100"
```

Starting 100 m off the pad, constant decel still lands on target (miss 0.03 m). Late burn misses by 32.8 m and arrives still moving sideways at 5.00 m/s. That figure is exactly `max_vx_cmd`, so the controller was still commanding its fastest sideways speed when the lander hit the ground.

The cause is that the engine is off for the first few seconds of the coast, and after that it runs near full thrust for only a short time. Tilting can only steer while the engine is producing thrust, so there is little time to move sideways. Starting right above the pad, late burn uses 29.20 kg, versus 29.69 kg from 50 m off, so steering sideways costs fuel as well.

### 6.3 Late burn is sensitive to how hard it plans to brake

```powershell
./build/lander.exe --run "guidance=late-burn burn_safety=0.7" --run "guidance=late-burn burn_safety=0.8" --run "guidance=late-burn burn_safety=0.9" --run "guidance=late-burn burn_safety=1.0"
```

| `burn_safety` | Result | Time | Landing speed | Fuel used |
| --- | --- | --- | --- | --- |
| 0.7 | success | 20.75 s | 1.30 m/s | 31.51 kg |
| 0.8 | success | 19.47 s | 1.44 m/s | 30.54 kg |
| 0.9 | success | 18.43 s | 1.71 m/s | 29.69 kg |
| 1.0 | FAIL | 15.40 s | 11.62 m/s | 23.31 kg |

A safety factor of 1.0 asks the lander to brake as hard as the engine can, which leaves the controller no spare thrust to correct errors, so it crashes. A lower factor trades a little fuel and time for a much softer landing.

Changing the engine also changes late burn's behaviour. With Moon gravity and `max_thrust=12000`, late burn fails at 9.82 m/s, and with `max_thrust=9000` it fails at 4.70 m/s. In both cases the planned deceleration is far larger than in the default setup. My explanation is that a velocity controller needs time to catch up with a fast-changing target, so the faster the target changes, the further behind the lander falls. I did not run an experiment that isolates this, so treat it as a hypothesis.

### 6.4 Stronger planned deceleration lands sooner, uses less fuel, lands harder

```powershell
./build/lander.exe --run "const_decel=0.7" --run "const_decel=1.0" --run "const_decel=1.5" --run "const_decel=2.0"
```

| `const_decel` | Time | Landing speed | Fuel used |
| --- | --- | --- | --- |
| 0.7 | 35.92 s | 0.89 m/s | 42.13 kg |
| 1.0 | 31.09 s | 0.85 m/s | 38.82 kg |
| 1.5 | 24.82 s | 1.08 m/s | 34.42 kg |
| 2.0 | 21.44 s | 1.22 m/s | 32.02 kg |

All four succeed. Time and fuel drop steadily as the planned deceleration grows. Landing speed rises, with one small exception: 0.7 lands slightly harder than 1.0, and I did not investigate that.

### 6.5 PID gains barely matter on the default scenario

```powershell
./build/lander.exe --run "kp_vert=1" --run "kp_vert=2" --run "kp_vert=4 ki_vert=0.4" --run "kp_vert=8"
```

| Gains | Time | Landing speed | Fuel used |
| --- | --- | --- | --- |
| kp 1 | 30.73 s | 1.00 m/s | 38.53 kg |
| kp 2 (default) | 31.09 s | 0.85 m/s | 38.82 kg |
| kp 4, ki 0.4 | 30.52 s | 0.93 m/s | 38.38 kg |
| kp 8 | 30.88 s | 0.76 m/s | 38.69 kg |

All four succeed, within 0.6 s, 0.25 m/s and 0.5 kg of each other. Constant deceleration is a gentle target, so the controller has an easy job. The gains would matter more for a harsher target, as in 6.3.

![Gain sweep](screenshots/gain_sweep.png)

### 6.6 Windup mattered

Late burn originally landed at 2.34 m/s, a failure. During the coast the engine is off but the PID was still accumulating error, and that stored error was dumped into the throttle when the engine came back on. Resetting the PIDs while the engine is off fixed it.

## 7. Limitations

- Point mass in 2D: there is no rotation, no angular dynamics and no engine gimbal lag. The lander tilts instantly to whatever angle it is commanded, up to the limit.
- Flat ground, constant gravity, no atmosphere.
- Perfect sensing: the controller sees the exact state, with no noise or delay.
- Mass flow is exactly proportional to throttle.
- Findings come from a small set of scenarios (the Moon and Mars settings, one start geometry plus a few variations). Treat them as observations about this simulator and not as general results about the guidance laws.
- Late burn fails outright when steering is needed from far off, or when it is set to brake at the engine's limit (see 6.2 and 6.3), so the fuel advantage in 6.1 holds only under easy conditions.

## 8. Reproducing

```powershell
New-Item -ItemType Directory -Force build, data
g++ -std=c++17 -Wall -Wextra src/main.cpp src/lander.cpp src/logger.cpp src/guidance.cpp src/pid.cpp src/controller.cpp src/touchdown.cpp src/config.cpp src/runner.cpp -o build/lander.exe
./build/lander.exe --run "guidance=constant-decel" --run "guidance=late-burn"
py scripts/plot_compare.py --save docs/screenshots/guidance_comparison.png
```