# cobot_py

Pure-Python driver for the Addverb cobot. No ROS 2, no C++ build, no third-party
packages — standard library only.

It speaks the exact protocol the C++ backend spoke: framed JSON (`#...#`) over a
TCP socket to the robot controller on port and ip configured in the ip.csv.

> This is the `python_examples` branch of [Syncro_5](https://github.com/HumanoidAddverb/Syncro_5).
> For the ROS 2 stack, use the `main` (Syncro) or `heal` branch.

## Setup first

Follow **[Setup.md](Setup.md)** before the first run. It covers:

1. Robot side: loading/running the docker image, FT calibration, starting the server.
2. Cobot backend: copying [cobot_backend/](cobot_backend/) to `/opt/addverb/` and setting the
   robot IP/port in `/opt/addverb/cobot_backend/config/ip.csv`.
3. Installing `cobot_py` and checking the connection with `python3 -m cobot.diagnose`.
4. Switching between Wi-Fi and Ethernet.

```bash
git clone -b python_examples git@github.com:HumanoidAddverb/Syncro_5.git
cd Syncro_5
sudo mkdir -p /opt/addverb/ && sudo cp -r cobot_backend /opt/addverb/
cd cobot_py && python3 -m pip install -e ".[dev]"
python3 -m cobot.diagnose
```

```python
from cobot import Cobot, API

with Cobot("192.168.0.12") as bot:
    bot.set_payload(mass=1.0, com=[0, 0, 0.05], moi=[0] * 6)
    bot.power_on()

    bot.move_to([0, 0, 0, 0, 0, 0], duration=10)

    with bot.control_stream(API.EXTERNAL_VELOCITY, rate_hz=100) as stream:
        for t in stream.ticks(seconds=5):
            bot.set_velocity([0, 0.1, 0, 0, 0, 0])

    print(bot.joint_positions)
    bot.power_off()
```

## Install

```bash
cd cobot_py
python3 -m pip install -e ".[dev]"
```

Editable install, so edits under `src/cobot/` take effect immediately — no
rebuild step. Check it:

```bash
python3 -c "import cobot; print(cobot.__version__)"
```

If you skip the install, the scripts in `examples/` still run: each one imports
`examples/_path.py` first, which puts `src/` on `sys.path`. Your own scripts
living outside `examples/` need the install (or `PYTHONPATH=/path/to/cobot_py/src`),
otherwise `import cobot` raises `ModuleNotFoundError`.

## When the connection will not come up

```bash
python3 -m cobot.diagnose                 # uses /opt/addverb config defaults
python3 -m cobot.diagnose 192.168.0.12 15263
```

It walks the connect sequence one stage at a time — deployed config, TCP reach,
handshake, first framed packet, decode — and says which stage failed.

Note the roles: the **robot is the TCP server** (`ROBOT_COMM_TYPE = eTCPServer`)
and this driver is the client.

### One connection per session — this bites

`SocketServer::initConnect` does a single `listen()`/`accept()` per session and
`CONNECTION_QUEUE` is **1**. Consequences:

* Never open a throwaway "is the port reachable" socket. It consumes the robot's
  only accept slot and the real connection then gets **`[Errno 104] Connection
  reset by peer`** or `Connection refused`. `cobot.diagnose` deliberately uses one
  connection for all of its stages; `tests/test_single_accept.py` enforces that.
* A run that dies mid-session can leave the robot never returning to `accept()`.
  **Restart the robot-side controller before retrying.**
* A leftover C++ backend or ROS 2 node holding the socket will lock you out.

### The handshake is a rendezvous, not a knock

Both ends run `establishDataLine_()` and each waits for the other. Our TCP connect
succeeds off the **listen backlog**, which can be long before the robot's
`DataManager::connect()` actually calls `accept()`. So the client resends the init
packet every `handshake_poll_interval` (0.5 s) for up to `handshake_timeout`
(10 s), and the whole connect is retried `connect_attempts` (30) times with
`connect_retry_delay` (2 s) between tries — mirroring `CobotHWInterface::connect_()`,
which loops up to `max_reattempt_connection_count` (100). That keeps a packet
queued for the instant the robot polls, and keeps reconnecting until it arrives.

Give up too early and the robot accepts a socket you already closed. A closed peer
is *readable with zero bytes*, so `ioctl(FIONREAD)` returns 0 and it logs, forever:

```
accepted
no data to read
attempted to establish data line
```

If you see that, the client quit before the robot got there. Restart the robot
controller and start the Python side first — it will wait.

### Read the robot-side log before suspecting this code

| Robot log | Meaning |
|---|---|
| `api recd from the other end : 18`, `Updated Robot configuration` | Our packet arrived and parsed. Link and protocol are fine. |
| `motion state object created` | The robot began the BASE → MOTION transition. |

To watch the transition live:

```bash
python3 -m cobot.diagnose 192.168.0.12 15263 --power-on --seconds 10
```

## Try it without hardware

```bash
python tools/fake_robot.py --port 15263          # terminal 1
COBOT_IP=127.0.0.1 python examples/02_move_to.py # terminal 2
python -m pytest tests/ -q
```

## Step-by-step walkthrough notebook

[`notebooks/robot_walkthrough.ipynb`](cobot_py/notebooks/robot_walkthrough.ipynb) walks
through a full session one cell at a time, from connecting to powering off:

| Step | Covers |
|---|---|
| 0 | Setup: imports, robot IP/port, optional simulator |
| 1 | Connect (the one TCP connection + handshake) |
| 2 | Read state and feedback |
| 3 | Configure safety mode, payload and gripper (before power on) |
| 4 | Power on (BASE → MOTION) |
| 5 | One-shot goals: `move_to`, `move_through` |
| 6 | Switch controllers and stream commands: velocity, joint jog, cartesian jog |
| 7 | Gripper open / close |
| 8 | Free drive → teach waypoints → replay |
| 9 | Errors and recovery: `clear_error()`, `recover()` |
| 10 | Power off and disconnect |

```bash
python3 -m pip install jupyter      # not part of the [dev] extras
jupyter notebook notebooks/robot_walkthrough.ipynb
```

Edit `ROBOT_IP` / `ROBOT_PORT` in the setup cell first, or set `SIMULATOR = True`
to have the notebook start `tools/fake_robot.py` and connect to it. It finds
`src/` itself, so the editable install is not required.

Run the cells **in order, top to bottom**, and run the connect cell **only once**:
a second connection would use up the robot's single accept slot (see
*One connection per session* above). Try it against the simulator before you
point it at real hardware.

## Repository layout

```
Syncro_5 (python_examples branch)/
├── README.md             this file
├── Setup.md              robot, backend and cobot_py setup
├── cobot_backend/        copy to /opt/addverb/ (see Setup.md)
│   ├── config/ip.csv     robot IP (line 1) and port (line 2)
│   ├── include/          backend headers
│   └── lib/              prebuilt backend libraries
└── cobot_py/             the Python driver (below)
```

## cobot_py layout

```
cobot_py/
├── pyproject.toml
├── src/cobot/
│   ├── __init__.py       public API surface
│   ├── config.py         IP/port/timeouts, joint + gripper limits
│   ├── enums.py          API ids, states, actions, events, error codes
│   ├── errors.py         exception hierarchy
│   ├── messages.py       SocketTx / SocketRx dataclasses + JSON codec
│   ├── transport.py      TCP socket + '#...#' framing + handshake
│   ├── link.py           background read/write pump thread
│   ├── session.py        TX/RX struct state, sticky vs volatile fields
│   ├── limits.py         local command validation
│   └── robot.py          Cobot — the class you actually use
├── examples/             runnable scripts, one concept each
├── notebooks/robot_walkthrough.ipynb   guided session, cell by cell
├── tools/fake_robot.py   protocol-compatible simulator
└── tests/test_smoke.py   end-to-end tests against the simulator
```

## How it maps to the C++ it replaces

Read this table if you are porting behaviour or debugging against the old stack.

| Python | Replaces | Notes |
|---|---|---|
| `transport.FramedSocket` | `socket_client.h`, `socket_communication_json.h`, `socket_comm_wrapper.h`, `socket_client_comm_wrapper.h` | Four C++ classes collapse into one: connect, `#`-framing, leftover buffering, and the data-line handshake. |
| `link.RobotLink` | `network_manager.h` | Same `read() -> send(latest_tx)` loop on a background thread. |
| `session.RobotSession` | `data_handler.h/.cpp` | Owns `SocketTx`/`SocketRx` and the clear policy. |
| `messages.py` | `ui_data_types.h` (1499 lines) | Dataclasses with the exact `struct_mapping::reg` JSON keys. |
| `enums.py` | `api_types.h`, `robot_fsm_enums.h`, `event_types.h`, `error_codes.h`, `robot_config_info.h` | |
| `config.py` | `socket_config.h`, `network_config.h`, `robot_config.h`, `backend/config/ip.csv` | |
| `limits.py` | `utility/src/data_validator.cpp` | |
| `robot.Cobot` | `utility/src/cobot_comm_wrapper.cpp` | Power on/off, controller switching, goals, events, error recovery. |
| — | `http_server.h`, `websocket_server.h`, `listener.h`, `data_parser.h`, `shared_state.h`, `backend_executor.h`, `data_store.h` | Dropped. These existed only to serve the Angular frontend over HTTP/WebSocket. Raw Python calls `Cobot` directly. |
| — | `utility/data_processor.h`, `data_converter.h`, `data_communicator.h`, `hardware_interface_defs.h` | Dropped. That chain existed to adapt `ros2_control` types onto `CobotCommWrapper`; with no ROS 2 there is nothing to adapt. |

Roughly 6000 lines of C++ across two repos become ~1200 lines of Python.

## The protocol, in one page

Everything below was read off the C++ implementation. If you extend the driver,
these are the invariants to preserve.

**Framing.** Every packet is `#` + JSON + `#`. Bytes before the first `#` are
discarded (`SocketCommWrapper::read`).

**Handshake.** The robot stays silent until it receives a valid packet. On connect
the client resends an idle `SocketTx` until data appears on the socket
(`establishDataLine_`), and retries the whole connect if that never happens. A TCP
accept alone means nothing — and "readable" alone means nothing either, since a
closed peer is readable with zero bytes. See the rendezvous section above.

**Cadence.** The robot streams state; every packet it sends must be answered with
the current command. That is why `RobotLink` runs a thread: application code sets
the next TX and the pump keeps the robot fed. The robot paces the loop, so the
thread blocks rather than spins.

**Nulls are significant on the wire.** The C++ serialises with
`map_struct_to_json(s, ss, " ", false)` — `hide_null = false`, so unset optionals
are emitted as explicit `null`, not omitted. `messages.py` reproduces this; do not
"optimise" it into skipping keys.

**Sticky vs volatile TX fields.** After every write the C++ clears
`interrupt.action`, `interrupt.error` and `control_int`, but leaves `config` and
`event_config` in place to be resent (`DataHandler::clearSendData_` vs
`clearData`). `RobotSession.write()` does the same. This is why a controller
config is staged once and then only the per-cycle command is pumped — and why
`_pump_until` takes a `rearm` callback to re-set the action each cycle.

**Two command shapes.**

- *One-shot goals* (`move_to`, `move_through`, `replay`): stage a
  `controller_config` plus a `RESET_CONFIG` event, wait for the robot to report
  `EXECUTING`, then withdraw both so they are not replayed.
- *Continuous controllers* (`EXTERNAL_VELOCITY`, `EXTERNAL_TORQUE`,
  `GCOMP_EXTERNAL_TORQUE`, `JOG_JOINT`, `JOG_RPY`, `FREE_DRIVE` — see
  `CONTINUOUS_APIS`): handshake through `EXECUTING` then `SUCCEEDED`, after which
  a `control_int` must arrive every cycle.

`API.EXTERNAL_TORQUE` is in `CRITICAL_APIS`: a valid torque must be streamed
*during* the switch or the arm drops. `switch_controller` refuses to switch to it
without a `first_command`.

## Deliberate differences from the C++

These are behaviour changes, not oversights:

1. **All waits are bounded.** The C++ transition loops (`powerOnRobot`,
   `switchController`, `setCmd`) are `while` loops with no timeout — a wedged
   robot hangs the process forever. Every wait here is bounded by
   `LinkConfig.command_timeout` (default 30 s) and raises `CommandTimeout`.
2. **Waits block on a fresh packet.** The C++ re-read the same buffered packet in
   a tight loop. `RobotLink` tracks a sequence number so a wait consumes each
   packet once.
3. **Failures raise.** No `error_codes` return value to forget to check. The code
   is still available as `.code` on the exception.
4. **Validation runs before the wire.** `limits.py` rejects out-of-range commands
   in Python rather than letting the robot fault into `ERROR`.
5. **`power_on()` propagates the safety mode as given.** The C++
   `powerOnRobot()` has no `return` on its final fall-through path (undefined
   behaviour if the state is neither BASE, MOTION nor ERROR); here that case
   raises.

## Where the numbers came from

Defaults in `config.py` are copied from the C++ and are worth re-checking against
your hardware:

- IP `192.168.0.12`, port `15263` — `backend/config/ip.csv`, `socket_config.h`.
  Override per run with `COBOT_IP` / `COBOT_PORT`, or `LinkConfig(ip=..., port=...)`.
- Joint velocity ±1.0 rad/s, effort ±60 Nm, jog ±3.0, gripper force 0–150 N,
  per-joint position limits — `utility/include/utility/robot_config_info.h`.
  That header carries the comment *"copied from robot end — must make sure
  manually to update this one"*, so treat the limits as a starting point.
- Safety mode `NO_HUMAN_SAFETY` (1) — `addverb_cobot_control/config/payload_config.yaml`.
  Valid modes are 0–2; `3` is `eMaxRegime` (an enum count) and the robot rejects
  it with `INVALID_SAFETY_TYPE`.
