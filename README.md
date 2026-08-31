# VEXcode summer testing
Codebase built on top of the VEX IDE, VEX code

# Competition Code for VEX robotics team 

# VEX V5 Control System Documentation

A guide to our PID control loops, motor multithreading, and driver control logic for the VEX V5 robot.

---

## 1. PID Control

The `PID` class handles closed-loop feedback for chassis movement and turning algorithms.

* **Target Tracking:** Updates error (`Target - Input`) to derive PID gains.
* **Turn Mode:** Uses `shortDeg()` to calculate the shortest path angle.
* **Integral Safety:** Limits integral buildup via range gating (`IRange`), and a integral max variable (`IMax`), with resets error on zero-crossings or when target tolerance is reached.
* **Settling Detection:** Marks target arrival (`targetArrived()`) when error stays within threshold tolerances for `jumpTime` or hits a hard 2000 ms timeout.

| Core PID Methods | Function |
| :--- | :--- |
| `setCoefficient(kp, ki, kd)` | Configures PID loop gain parameters. |
| `setTarget(target)` | Assigns target position or angle setpoint. |
| `update(input)` | Calculates P, I, D outputs and handles error timers. |
| `getOutput()` | Returns total control output (P + I + D). |

---

## 2. Motor Control

Manages hardware tasks with threads:

* **Multithreading:** Creates background threads in `main()`.
* **Joystick filtering:** Filters out small joystick movements below `Joystick_LowerDeadzone`.
* **Actuation Sync:** Coordinates pneumatic solenoids (`setPistonA`–`W`) alongside motor speed outputs.

---

## 3. Driver Control 

Maps V5 Controller inputs to Arcade Drive movement, button presses, and motor priority logic during human control.

* **Chassis Motion:** Single-stick Arcade Drive mixing Axis 3 (forward) and Axis 1 (turn):
  * **Left Velocity:** `Ch3 + 0.6 * Ch1`
  * **Right Velocity:** `Ch3 - 0.5 * Ch1`
* **Auto-Collision Override:** Forces fixed motor velocity vectors (`autocollide * 15`) when active.

### Example from our 2025/2026 VEX Push Back Season on 5327K

| Inputs | Intake Speed | Pneumatics / Lift State |
| :--- | :---: | :--- |
| **L1 + R1** (Hold) | `0%` | Piston O: `true`, Piston D: `true`, Lift: `false` |
| **R1** | `100%` | Piston D: `true`, Lift: `true` |
| **R2** | `-100%` | Piston D: `true`, Low State: `true` |
| **L1** | `100%` | Piston D: `false`, Lift: `false` |
| **L2** | `100%` | Piston D: `true`, Lift: `false` |
| **None** (Idle) | `0%` | Piston D: `true`, Lift: `false` |

### Controller Button Mapping

* **B:** Toggles match-load pneumatic piston (`setPistonI`).
* **X / Right:** Toggles right and left mid-wing pneumatics (`setPistonG` / `setPistonF`).
* **A:** Toggles manual `lowstate` mode.
* **Down / Up:** Manages `setPistonW` and flags autonomous routines complete.

---  
### Upcomming competetion robot and future plans:

* We continue to apply this code to our new competition robot, for the Speedway Signature event.

<img width="1022" height="750" alt="Screenshot 2026-08-30 at 11 32 49 PM" src="https://github.com/user-attachments/assets/60835866-4bab-419c-8a36-aeca781b16e6" />
