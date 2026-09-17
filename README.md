# ChalaoOS

**A robot operating environment whose services are written in [Robot Chalao](https://github.com/megazron/robot-chalao) — the Hinglish robot language.** One manifest boots your whole robot: init, a service supervisor, a scheduler, a hardware abstraction layer, an IPC bus, an e-stop, and a shell. Built on the Robot Chalao native C++ core.


<p align="center"><img src="docs/img/boot.gif" width="100%" alt="ChalaoOS boot demo: services start in dependency order, publish over the bus, and a watchdog trips the e-stop"></p>

<p align="center"><i>One command boots the robot: <code>chalaoos --demo</code>. Services start in order, publish over the bus, and the watchdog trips the e-stop on low battery.</i></p>

![ChalaoOS architecture](docs/img/architecture.svg)

```bash
make
./chalaoos --demo --seconds 8
```

## What it is, and what it is not

ChalaoOS is a robot **operating environment** — a runtime that boots a robot and keeps its programs alive — in the same sense that "ROS" (Robot Operating System) is a runtime, not a kernel. It is **not** a bare-metal OS kernel and does not replace Linux. What it gives you is the layer a robot actually needs on top of the OS:

- **init + boot** from a single system manifest,
- a **supervisor** that starts services in dependency order and restarts them by policy,
- a **scheduler** that runs periodic services at their rate,
- a **HAL** (hardware abstraction layer) with an **e-stop** every service can trip,
- an **IPC bus** (blackboard) for services to publish and read topics,
- a **shell** to inspect and control the running system,

and the thing that makes it different from ROS: **every service is a Robot Chalao `.rc` program**, and the whole system is described by one readable manifest.

## Boot demo

`./chalaoos --demo --seconds 8` boots the bundled patrol robot and runs a deterministic story:

```
  +--------------------------------------------------+
  |   ChalaoOS 0.1.0  --  robot operating environment |
  |   services ki zubaan: Robot Chalao (.rc)         |
  +--------------------------------------------------+

[  0.00s] init       | boot: 5 service(s) in manifest
[  0.00s] init       | starting 'bringup'  (never, once)
[  0.00s] bringup    | patrolbot system online
[  0.00s] init       | starting 'heartbeat'  (on-failure, 2.0 Hz)
...
[  0.50s] heartbeat  | dhadkan: battery 98 percent
[  0.50s] drive      | [hal] wheels drove 1.0 m (odom 1.0)
...
[  6.00s] watchdog   | battery kam hai -- e-stop!
[  6.00s] kernel     | *** E-STOP asserted by service 'watchdog' -- actuators cut ***
[  7.00s] drive      | [hal] BLOCKED -- actuators are in e-stop
...
  ChalaoOS band. Alvida.
```

Boot in dependency order, the scheduler ticking heartbeat / perception / drive, the battery draining through the HAL, the `watchdog` service tripping the **e-stop** when the battery gets low, the HAL then refusing to move the wheels, and a clean halt with a `ps` table and the topic bus.

## Architecture

```
                         ChalaoOS
  +---------------------------------------------------------+
  |  shell   ( ps | start/stop/restart | log | echo | ...)  |
  +----------------------------+----------------------------+
  |  supervisor  (dep order,   |  scheduler (virtual clock, |
  |  restart policy, state)    |  per-service rate_hz)      |
  +----------------------------+----------------------------+
  |            embedded Robot Chalao interpreter            |
  |   one per service  --  outSink -> logs, robotHook -> HAL|
  +------------------+------------------+-------------------+
  |   HAL (battery,  |   IPC bus        |   e-stop          |
  |   wheels, sensors)|  (blackboard)   |   (safety cut)    |
  +------------------+------------------+-------------------+
        ^ services are .rc programs: bringup, heartbeat,
          perception, drive, watchdog  (system/*.rc)
```

Each service gets its own interpreter. Two hooks connect the language to the OS: an **output sink** routes every `dikhao`/sim line into the service's log, and a **robot-command hook** routes commands like `battery kitni hai`, `aage chalo`, `bolo` and `band karo` into the HAL, the bus, and the e-stop. Commands the OS doesn't intercept fall back to the language's built-in simulation.

## The manifest

`system/system.manifest` describes the robot. Each `[service ...]` block:

```ini
[service heartbeat]
script    = heartbeat.rc      # a Robot Chalao program
after     = bringup           # start after these services
rate_hz   = 2                 # run 2x/second (0 = run once at boot)
restart   = on-failure        # never | on-failure | always
autostart = true
```

## Writing a service

A service is ordinary Robot Chalao. The watchdog, in full:

```
# battery kam ho toh poora system band karo
maano b = battery kitni hai
agar b < 78 toh
  dikhao "battery kam hai -- e-stop!"
  band karo
khatam
```

`battery kitni hai` reads the HAL battery, and `band karo` trips the system e-stop — which the HAL then enforces on every actuator. The heartbeat publishes to the bus:

```
maano b = battery kitni hai
bolo "battery" b
```

## Shell

Run `./chalaoos` (no `--demo`) for an interactive shell after boot:

| command | does |
|---|---|
| `ps` | services, states, restarts, uptime |
| `topics` / `echo TOPIC` | list the bus / print a topic's last value |
| `start` / `stop` / `restart NAME` | control a service |
| `log NAME` | recent output of a service |
| `tick [n]` | advance the virtual clock n steps |
| `estop` | assert the e-stop |
| `uptime` | clock + battery |
| `halt` | stop everything and exit |

## Build and test

```bash
make            # -> ./chalaoos  (g++ -std=c++17, no dependencies)
make test       # boot, lifecycle, IPC, e-stop, restart-policy and boot-order tests
./chalaoos --demo --seconds 8
./chalaoos                       # interactive shell
./chalaoos system/system.manifest   # boot a specific manifest
```

Requires only a C++17 compiler. No ROS, no external libraries.

## Relationship to Robot Chalao

ChalaoOS embeds the **Robot Chalao native C++ core** (vendored in [`vendor/chalao/`](vendor/chalao/)). The language is the general-purpose way to program one robot; ChalaoOS is the runtime that boots and supervises many Robot Chalao programs as one system. See the language at <https://github.com/megazron/robot-chalao>.

## Roadmap

- Preemptive scheduling and per-service CPU/time budgets.
- Persistent state and a proper `sun` (subscribe) delivery path from the bus into services.
- A real hardware backend behind the HAL (ROS 2 / serial) so the same manifest runs on a physical robot.
- Health checks and liveness probes per service; a `journalctl`-style log store.
- A boot-time dependency graph visualiser.

## License

MIT — see [LICENSE](LICENSE).
