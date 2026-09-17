# zronOS x MuJoCo compatibility matrix

**68 / 68** open-source MuJoCo Menagerie robot models boot and run under zronOS through the bridge (real MuJoCo physics).

Each model is loaded in MuJoCo, introspected for its actuators, driven by an auto-generated Robot Chalao service under the real `zron` binary, and stepped 200 times through physics with the setpoints zron commanded. A row passes only if MuJoCo loaded it, zron booted the driver, the setpoint count matched the actuator count, and the state stayed finite.

## By category

| category | pass / total |
|---|---|
| arm | 21 / 21 |
| drone | 3 / 3 |
| hand | 9 / 9 |
| humanoid | 14 / 14 |
| mobile | 6 / 6 |
| other | 6 / 6 |
| quadruped | 9 / 9 |

## All models

| model | category | actuators | DOF | sensors | MuJoCo load | zron boot | setpoints | physics | result |
|---|---|---|---|---|---|---|---|---|---|
| `agilex_piper` | arm | 7 | 8 | 0 | yes | yes | yes | yes | PASS |
| `arx_l5` | arm | 7 | 8 | 0 | yes | yes | yes | yes | PASS |
| `flexiv_rizon4` | arm | 7 | 7 | 0 | yes | yes | yes | yes | PASS |
| `flexiv_rizon4s` | arm | 7 | 7 | 14 | yes | yes | yes | yes | PASS |
| `franka_emika_panda` | arm | 8 | 9 | 0 | yes | yes | yes | yes | PASS |
| `franka_fr3` | arm | 7 | 7 | 0 | yes | yes | yes | yes | PASS |
| `franka_fr3_v2` | arm | 7 | 7 | 0 | yes | yes | yes | yes | PASS |
| `i2rt_yam` | arm | 7 | 8 | 0 | yes | yes | yes | yes | PASS |
| `kinova_gen3` | arm | 7 | 7 | 0 | yes | yes | yes | yes | PASS |
| `kuka_iiwa_14` | arm | 7 | 7 | 0 | yes | yes | yes | yes | PASS |
| `low_cost_robot_arm` | arm | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `rethink_robotics_sawyer` | arm | 7 | 7 | 0 | yes | yes | yes | yes | PASS |
| `robotstudio_so101` | arm | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `seeed_rebot_devarm` | arm | 8 | 8 | 0 | yes | yes | yes | yes | PASS |
| `trossen_vx300s` | arm | 7 | 8 | 0 | yes | yes | yes | yes | PASS |
| `trossen_wxai` | arm | 14 | 16 | 0 | yes | yes | yes | yes | PASS |
| `trs_so_arm100` | arm | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `ufactory_lite6` | arm | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `ufactory_xarm7` | arm | 8 | 13 | 0 | yes | yes | yes | yes | PASS |
| `universal_robots_ur10e` | arm | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `universal_robots_ur5e` | arm | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `bitcraze_crazyflie_2` | drone | 4 | 6 | 3 | yes | yes | yes | yes | PASS |
| `skydio_x2` | drone | 4 | 6 | 3 | yes | yes | yes | yes | PASS |
| `trossen_wx250s` | drone | 7 | 8 | 0 | yes | yes | yes | yes | PASS |
| `iit_softfoot` | hand | 1 | 93 | 0 | yes | yes | yes | yes | PASS |
| `leap_hand` | hand | 16 | 16 | 16 | yes | yes | yes | yes | PASS |
| `robotiq_2f85` | hand | 1 | 14 | 0 | yes | yes | yes | yes | PASS |
| `robotiq_2f85_v4` | hand | 1 | 12 | 0 | yes | yes | yes | yes | PASS |
| `shadow_dexee` | hand | 12 | 12 | 12 | yes | yes | yes | yes | PASS |
| `shadow_hand` | hand | 20 | 30 | 0 | yes | yes | yes | yes | PASS |
| `sharpa_wave` | hand | 22 | 22 | 0 | yes | yes | yes | yes | PASS |
| `tetheria_aero_hand_open` | hand | 7 | 16 | 7 | yes | yes | yes | yes | PASS |
| `umi_gripper` | hand | 7 | 8 | 0 | yes | yes | yes | yes | PASS |
| `apptronik_apollo` | humanoid | 32 | 38 | 4 | yes | yes | yes | yes | PASS |
| `berkeley_humanoid` | humanoid | 12 | 18 | 13 | yes | yes | yes | yes | PASS |
| `booster_t1` | humanoid | 23 | 29 | 3 | yes | yes | yes | yes | PASS |
| `fourier_n1` | humanoid | 23 | 29 | 3 | yes | yes | yes | yes | PASS |
| `ms_human_700` | humanoid | 700 | 85 | 0 | yes | yes | yes | yes | PASS |
| `pal_talos` | humanoid | 32 | 50 | 0 | yes | yes | yes | yes | PASS |
| `pndbotics_adam_lite` | humanoid | 25 | 31 | 0 | yes | yes | yes | yes | PASS |
| `rainbow_robotics_rby1` | humanoid | 26 | 34 | 0 | yes | yes | yes | yes | PASS |
| `robotis_op3` | humanoid | 20 | 26 | 0 | yes | yes | yes | yes | PASS |
| `stanford_tidybot` | humanoid | 11 | 18 | 0 | yes | yes | yes | yes | PASS |
| `toddlerbot_2xc` | humanoid | 30 | 50 | 0 | yes | yes | yes | yes | PASS |
| `toddlerbot_2xm` | humanoid | 30 | 50 | 0 | yes | yes | yes | yes | PASS |
| `unitree_g1` | humanoid | 29 | 35 | 4 | yes | yes | yes | yes | PASS |
| `unitree_h1` | humanoid | 19 | 25 | 0 | yes | yes | yes | yes | PASS |
| `google_robot` | mobile | 9 | 9 | 0 | yes | yes | yes | yes | PASS |
| `hello_robot_stretch` | mobile | 8 | 29 | 0 | yes | yes | yes | yes | PASS |
| `hello_robot_stretch_3` | mobile | 10 | 38 | 2 | yes | yes | yes | yes | PASS |
| `pal_tiago` | mobile | 14 | 28 | 0 | yes | yes | yes | yes | PASS |
| `pal_tiago_dual` | mobile | 25 | 31 | 0 | yes | yes | yes | yes | PASS |
| `robot_soccer_kit` | mobile | 4 | 70 | 0 | yes | yes | yes | yes | PASS |
| `aloha` | other | 14 | 16 | 0 | yes | yes | yes | yes | PASS |
| `dynamixel_2r` | other | 2 | 2 | 0 | yes | yes | yes | yes | PASS |
| `flybody` | other | 78 | 108 | 15 | yes | yes | yes | yes | PASS |
| `realsense_d435i` | other | 0 | 0 | 0 | yes | yes | yes | yes | PASS |
| `unitree_z1` | other | 6 | 6 | 0 | yes | yes | yes | yes | PASS |
| `wonik_allegro` | other | 16 | 22 | 0 | yes | yes | yes | yes | PASS |
| `agility_cassie` | quadruped | 10 | 32 | 20 | yes | yes | yes | yes | PASS |
| `anybotics_anymal_b` | quadruped | 12 | 18 | 0 | yes | yes | yes | yes | PASS |
| `anybotics_anymal_c` | quadruped | 12 | 18 | 0 | yes | yes | yes | yes | PASS |
| `boston_dynamics_spot` | quadruped | 12 | 18 | 0 | yes | yes | yes | yes | PASS |
| `google_barkour_v0` | quadruped | 12 | 18 | 27 | yes | yes | yes | yes | PASS |
| `google_barkour_vb` | quadruped | 12 | 18 | 30 | yes | yes | yes | yes | PASS |
| `unitree_a1` | quadruped | 12 | 18 | 0 | yes | yes | yes | yes | PASS |
| `unitree_go1` | quadruped | 12 | 18 | 0 | yes | yes | yes | yes | PASS |
| `unitree_go2` | quadruped | 12 | 18 | 0 | yes | yes | yes | yes | PASS |
