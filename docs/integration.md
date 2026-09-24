# Контракт SFERA ↔ sfera-target-sim

Стан на 2026-09-24: SFERA `5b4e912` (гілка `refactor/agent-friendly`), симулятор `63a74df` (`refactor/split-modules`).
Усе нижче звірено з кодом обох репозиторіїв і прогоном `tools/joint_check.py`. Першоджерела:
`../sfera/docs/interfaces.md` і `../sfera-target-sim/docs/protocol.md`.

## Схема

```
sfera-target-sim                                   SFERA
  дрон 1  tcpin 127.0.0.1:5760  ◄── MAVLink ──►  D1: tcp:127.0.0.1:5760 (або COM-пара)
  дрон 2  tcpin 127.0.0.1:5762  ◄── MAVLink ──►  D2: tcp:127.0.0.1:5762
  ціль    UDP → 127.0.0.1:14551  ── CSV ~10 Гц ─►  TargetUDPSource, uid "UDP"
                                                  міст Mission Planner: TCP 0.0.0.0:5780
```

## Порти

| Порт | Хто слухає | Примітка |
|---|---|---|
| TCP 5760, 5762, … (крок 2) | симулятор, дрони | SFERA — клієнт: у UI поле `tcp:host:port` під списком COM |
| UDP 14551 | SFERA | поле «UDP сим.»; використовується лише перший порт зі списку |
| TCP 5780 | SFERA, міст MP | до `5b4e912` був 5760 і конфліктував із дроном 1; `0` вимикає |
| TCP 5760/5762/5763 (+10 на екземпляр) | ArduPilot SITL | стенд `../sfera/tools/sitl` |

## MAVLink

Симулятор прикидається ArduCopter (`MAV_TYPE_QUADROTOR`, `MAV_AUTOPILOT_ARDUPILOTMEGA`), тож `mode_mapping()` у SFERA
дає номери режимів ArduCopter (GUIDED = 4, LAND = 9).

| SFERA надсилає | Симулятор |
|---|---|
| `HEARTBEAT` (GCS) 1 Гц | ігнорує |
| `SET_MODE` GUIDED (часто, `ensure_guided`) | режим GUIDED, без ACK |
| `COMMAND_LONG ARM_DISARM` | ARM/DISARM + ACK |
| `COMMAND_LONG NAV_LAND` | посадка, на землі — DISARM |
| `SET_POSITION_TARGET_GLOBAL_INT`, `GLOBAL_RELATIVE_ALT_INT`, маска позиції / швидкості | позиція (alt = над HOME) / швидкість NED; зліт — `vz < 0` |
| `SET_ATTITUDE_TARGET` | кути + тяга |
| `SET_MESSAGE_INTERVAL`, `REQUEST_DATA_STREAM` | ACK, але частоти фіксовані |

Симулятор шле `HEARTBEAT` 1 Гц, `GLOBAL_POSITION_INT`/`HOME_POSITION`/`ATTITUDE` 10 Гц, `VFR_HUD`/`GPS_RAW_INT` 5 Гц.
SFERA не використовує `NAV_TAKEOFF` і `SET_MODE RTL`, яких симулятор по-справжньому не виконує (RTH у SFERA —
позиційна точка на HOME).

## UDP-цілі

Симулятор шле `lat,lon,alt_msl,azimuth,speed,timestamp\n` (7/7/2/2/2/3 знаки, timestamp — Unix-час, с).
SFERA приймає цей формат і скорочений `lat,lon,alt_msl,timestamp` (ISO або epoch). `azimuth`/`speed` SFERA лише
зберігає (`az_udp`/`spd_udp`) — швидкість цілі оцінює сама з різниці координат. `alt_msl` автопілот бере як цільову
висоту над рівнем моря; висоти узгоджені через HOME дрона (MSL у `HOME_POSITION`).

## Відомі розбіжності (не виправлено)

| # | Що | Наслідок |
|---|---|---|
| 1 | SFERA дає всім UDP-записам uid `UDP` (інваріант SFERA №6) | кілька цілей симулятора на одному порту зливаються в одну, що стрибає |
| 2 | Симулятор `max_climb_mps = 70`, SFERA відкидає `relative_alt`, що змінюється > 35 м/с (`ALT_REL_MAX_VZ_MPS`) | зараз не спрацьовує (RTH набирає ≤ 30 м), але позиційна точка з різницею висот > ~39 м заморозить `alt_rel` у SFERA |
| 3 | `../sfera/docs/interfaces.md` називає 6-польовий UDP-формат «застарілим», симулятор — контрактом; приклад у `../sfera/README.md` має 4 поля під 6-польовим заголовком | ризик, що хтось прибере 6-польову гілку парсера |
| 4 | Закомічений `simulator_config.json`: дрон 1 — `COM54`, є зайвий «Дрон 3»; README симулятора обіцяє дрон 1 на TCP 5760 | швидкий старт із README не збігається з конфігом |
| 5 | Симулятор ігнорує `SET_MESSAGE_INTERVAL` | `GPS_RAW_INT` 5 Гц замість 2, `HOME_POSITION` 10 Гц замість 0,5 — нешкідливо |
