# SITL-відтворення апарата з логу `00000086.BIN`

Мета: максимально близько відтворити в SITL апарат, який летів і впав. Лог — `00000086.BIN` у цій теці (копія з `~/Downloads/ArduPilot/`).

## Апарат (з логу)

- **Прошивка:** ArduPlane **V4.6.3**, git `3fc7011a`, ціль `MatekH743-bdshot` (Matek H743-WING V3).
- **Схема:** copter-style **quad tailsitter**, без керованих поверхонь.
  - `Q_ENABLE=1`, `Q_FRAME_CLASS=1`, `Q_FRAME_TYPE=1` (X).
  - `Q_TAILSIT_ENABLE=2` (Enable Always: Qassist завжди активний, airmode, для апаратів без поверхонь).
- **Мотори:** 4 шт., DShot300. Фізично на виходах 1–4 з функціями 34, 35, 36, 33.
- **Плата:** `AHRS_ORIENTATION=35` (ROLL_270_PITCH_270).
- **Батарея:** 8S HV (34.9 В на старті), 15 Аг. Струм у висінні ~15.8 А при 32.1 В.
- **Датчики:** GPS і компас DroneCAN (MatekG474-GPS), датчик повітряної швидкості MS4525 з `ARSPD_USE=0`.
- **Керування:** пульт CRSF/ELRS. `FLTMODE_CH=7`, `RC2_REVERSED=1`.
- **Канали опцій:** RC5 = arm/disarm (153), RC6 = нахил підвісу (213), RC11/RC12 = реле.
- **Маса:** 6–8 кг, центр мас — центр рами, трохи до носа (від власника, 2026-09-26); у моделі 7 кг, зміщення ЦМ SITL не моделює. `~/Downloads/ArduPilot/інструкція.md` описує **інший** літак (7 кг, один мотор, 12S), для цього апарата вона не підходить.

## Файли

| Файл | Що це |
|---|---|
| `00000086.BIN` | Вихідний лог польоту (DataFlash, 9 МБ). |
| `00000086_flight.param` | Точний знімок усіх 1514 параметрів із логу, без змін. Це еталон. |
| `00000086_sitl.param` | Параметри для SITL (опис нижче). |
| `tailsitter_00000086.json` | Модель рами SITL: напруга, струм, опір батареї, газ висіння та spin з логу, **відкалібрована в SITL** (`hoverThrOut 0.325`, `refCurrent 10.5`). **Маса — оцінка.** |
| `profile_bench.parm` | Профіль `bench`: `as-flown` + фейлсейфи, `Q_GUIDED_MODE=1`, реалістичні `AIRSPEED_*`. Кожен рядок пояснено. |
| `vehicle.json` | Профіль апарата для стенду SFERA (`sfera/tools/sitl`): бінарник, рама, параметри, профілі `as-flown`/`bench`, дім, дані калібрування. |
| `HARDWARE_GATES.md` | «Ворота» для реального апарата: що зробити до першого автономного FW. |
| `sitl_copter_tailsitter_x.patch` | 1-рядковий патч SITL, щоб модель copter tailsitter мала X-розкладку моторів. |
| `00000086_rc_input.csv` | RCIN з логу, 25 Гц. Стовпці `t_s, TimeUS, mode, armed, C1..C14`. Значення сирі, до `RC2_REVERSED`. |

### Що змінено в `00000086_sitl.param` відносно логу

- **Прибрано (111 параметрів):** калібрування (`INS_*OFFS/SCAL`, `COMPASS_OFS/DIA/ODI/MOT`, `AHRS_TRIM_*`, `ARSPD_OFFSET`, `BARO*_GND_PRESS`, `COMPASS_DEC`), ID датчиків, `*_CALTEMP`, `INS_TCAL*`, `STAT_*`, `FORMAT_VERSION`, `*_TOTAL`. Ці значення SITL генерує сам.
- **Замінено залізо на SITL:** блок `SITL overrides` у кінці файлу, у кожного рядка є коментар з оригінальним значенням.
  - GPS: DroneCAN замінено на серійний `SERIAL3`.
  - CAN вимкнено.
  - Датчик повітряної швидкості: `ARSPD_TYPE=100`.
  - Піни та масштаби батареї переведено на SITL.
  - `RC_PROTOCOLS=1`.
  - SERIAL4/5 вимкнено.
  - `Q_M_PWM_TYPE=0`.
  - `RELAY1/2_PIN=-1` (піни 81/82 у SITL не існують і блокують армінг), `COMPASS_USE2/3=0` (на апараті один компас DroneCAN; внутрішні компаси SITL розбіжні).
- **Мотори перенесено на виходи 5–8** з функціями 33–36. Модель SITL QuadPlane (`SIM_QuadPlane.cpp`, `motor_offset=4`) читає VTOL-мотори саме з виходів 5–8, а 1–4 вважає поверхнями літака. На апараті поверхонь немає, але `FUNCTION=0` на SERVO1–4 дає PWM 0, який `SIM_Plane` читає як газ −1 (реверс, +20 А) і повне відхилення поверхонь — апарат не злітає. Тому SERVO1–4 мають `FUNCTION=1` (passthrough центрованих RC1/2/4 і RC3=1000): «поверхонь і маршового мотора немає».
- **Орієнтація `SIM_IMU_ORIENT=0`, `AHRS_ORIENTATION=0`** (на апараті `AHRS_ORIENTATION=35`). Пара 35/35 не компенсується, а подвоює поворот: крен −90° на землі. Компас зовнішній, `COMPASS_ORIENT=0`.
- **Усе інше, зокрема весь тюнінг, без змін:** `Q_A_*`, `Q_P_*`, `Q_M_SPIN_MAX=0.6`, `Q_M_THST_HOVER=0.35` при `Q_M_HOVER_LEARN=0`, `Q_TAILSIT_*`, `THR_MAX=98`, `RC2_REVERSED=1`, режими та фейлсейфи.

## Запуск

### 1. Збірка точної версії 4.6.3

Робоча копія `~/ardupilot/ardupilot` стоїть на іншій гілці (4.7-dev, ESP32) і має незбережені зміни. Її **не чіпати**, а зробити окремий worktree:

```bash
cd ~/ardupilot/ardupilot
git worktree add ../ardupilot-4.6.3 3fc7011a
cd ../ardupilot-4.6.3
git submodule update --init --recursive
git apply /home/anikkhoma/repos/sfera/sfera-workspace/sims/SITL_00000086/sitl_copter_tailsitter_x.patch
```

### 2. Старт SITL через стенд SFERA (основний шлях)

Стенд `sfera/tools/sitl` запускає `arduplane` без MAVProxy за профілем `vehicle.json` (з кореня репозиторію `sfera`):

```bash
# лише SITL (SERIAL0 tcp:5760 — для SFERA/MP, SERIAL1 tcp:5762 — бічний канал стенду)
tools/sitl/run_sitl.sh start --vehicle ../sfera-workspace/sims/SITL_00000086/vehicle.json --profile bench
tools/sitl/run_sitl.sh stop
# сценарій зліт → висіння 30 с → QLAND із перевіркою калібрування (~2.5 хв)
.venv/bin/python -m tools.sitl.e2e --code-root . --vehicle ../sfera-workspace/sims/SITL_00000086/vehicle.json \
    --profile bench --scenario vtol_hover --out /tmp/sitl/vtol_hover.json
```

- Профіль `as-flown` — параметри як у лозі (плюс лише SITL-виправлення), `bench` — `as-flown` + `profile_bench.parm`.
- Стенд копіює модель у каталог екземпляра і передає її **відносним** шляхом: SITL відкидає початковий `/`, тож абсолютний шлях дає PANIC.
- Стенд вмикає `SERIAL1_PROTOCOL=2` (на апараті −1) для свого бічного каналу.
- З `FS_GCS_ENABL=1` (`bench`) апарат одразу після старту без HEARTBEAT від SFERA переходить у QLAND («GCS Failsafe On»), і армінг неможливий, поки SFERA не підключиться.

### 3. Старт SITL вручну через `sim_vehicle.py` (MAVProxy)

Точка Home і курс узято з логу (ORGN, ATT.Yaw ≈ 118°). Модель треба покласти в каталог запуску і дати відносний шлях:

```bash
mkdir -p /tmp/ts86 && cd /tmp/ts86
cp /home/anikkhoma/repos/sfera/sfera-workspace/sims/SITL_00000086/tailsitter_00000086.json model.json
~/ardupilot/ardupilot-4.6.3/Tools/autotest/sim_vehicle.py -v ArduPlane \
  -f quadplane-copter_tailsitter-x:model.json \
  --custom-location=49.9480133,25.5002564,271.36,118 \
  --add-param-file=/home/anikkhoma/repos/sfera/sfera-workspace/sims/SITL_00000086/00000086_sitl.param \
  -w --console --map
```

- У шляху до `.json` не повинно бути підрядків `-tri`, `-tilt`, `-plus`, `-hexa`, `-octa`, `-y6`, бо `SIM_QuadPlane` розбирає рядок рами через `strstr`.
- **Без патча:** використай `-f quadplane-copter_tailsitter:<json>` і додай `Q_FRAME_TYPE,0`. Це «+»-рама, тобто вже **не** та геометрія, що на апараті.
- Вітер (необов'язково): `param set SIM_WIND_SPD 4.2`, `param set SIM_WIND_DIR 88`. Це оцінка EKF (XKF2) лише за останні секунди польоту, у висінні вітер не оцінювався, бо `ARSPD_USE=0`.

### 4. Калібрування моделі

Злетіти (QLOITER або GUIDED + `NAV_TAKEOFF`) і висіти 20–30 с. У логу SITL перевірити:

| Величина | Ціль (лог) |
|---|---|
| `QTUN.ThO` | ≈ 0.57 (діапазон 0.49–0.65) |
| `RCOU.C5..C8` у середньому | ≈ 1486 |
| `BAT.Curr` | ≈ 15.8 А |

Якщо значення не збігаються, підправити `hoverThrOut` (і за потреби `mass`) у JSON та перезапустити. Коли стане відома справжня маса апарата, записати в JSON `mass = маса_кг / 1.5`.

## Що відтворювати: хронологія польоту

Час `t` відраховується від початку логу. Лог почався в момент армінгу (перемикач RC5, `ARM.Method=2`), апарат стояв на землі.

| t, с | Подія |
|---|---|
| 0 | Armed, QSTABILIZE. Висота 0, курс ≈ 118°. |
| ~5–15 | Зліт у QSTABILIZE. |
| 19.4 | QLOITER (ch7 = 1500, `FLTMODE4=19`). Набір висоти до ~36 м, стабільне висіння. |
| 69.4 | **FBWA** (ch7 = 2000). Стік тангажу C2 до 2000 (сирий, з урахуванням `RC2_REVERSED`). Апарат нахиляється вперед до −57°. |
| 73.2 | `Transition FW done`. |
| 73.6–77.2 | Pitch тримається +35…+25° при бажаному ≈ 0°, крен 10–19°. Висота падає з 38 до 21 м, швидкість відносно землі зростає до 25 м/с. **`RCOU` мотора з функцією 34 (Motor2) весь час на 1600 = `Q_M_SPIN_MAX` 0.6, тобто в насиченні.** |
| 77.2 | QLOITER. Газ C3 до максимуму (2010), бажаний pitch зростає до 50°, фактичний 15–33°. Зниження ~8 м/с при 25 м/с відносно землі. |
| 79.7 | `Transition VTOL done`. |
| 79.8 | Кінець логу, ще armed, висота ≈ −1.6 м від Home. Ймовірний удар об землю. |

### Відтворення дій пілота

Щоб відтворити дії пілота, подавати `00000086_rc_input.csv` у SITL як RC override з частотою 25 Гц. Підійде MAVProxy `rc` або pymavlink `RC_CHANNELS_OVERRIDE` на порт 5760/5762.

- Значення в CSV сирі, тому `RC2_REVERSED=1` у параметрах **має лишатися**.
- Колонка `mode` лише для контролю: режим сам перемикається каналом 7.
- RC5 (arm) у CSV має відповідне значення, але армінг у SITL краще робити після проходження pre-arm перевірок.

## Відомі межі точності

1. **Аеродинаміка літака:** у SITL це загальна модель `SIM_Plane`, а не це крило і фюзеляж. Поведінка після переходу у FW буде відрізнятися найбільше. Параметри опору (`refSpd`, `refAngle`, `disc_area`, `mdrag_coef`) невідомі, тому стоять значення за замовчуванням.
2. **Маса 6–8 кг (у моделі 7), діагональ і інерція невідомі:** інерція моделі — з `diagonal_size 0.5` (див. JSON).
3. **Датчик повітряної швидкості в SITL «чистий»:** на апараті він стоїть у потоці від гвинтів і шумить (6–7 м/с у висінні). Оскільки `ARSPD_USE=0`, на керування це не впливає.
4. **Вихідні сигнали:** DShot/bdshot замінено на PWM. Телеметрії ESC у логу немає.
5. **`SERVO12` (нахил підвісу) і реле** лишилися в параметрах, але фізики вони в SITL не мають.

## Використання зі SFERA

Стенд `../../../sfera/tools/sitl/` підтримує цей апарат через `vehicle.json` (див. «Старт SITL через стенд SFERA»).
Сценарій `vtol_hover` перевіряє калібрування (у SITL 2026-09-26: `ThO` 0.571, RCOU 1474, 15.6 А) і посадку QLAND.
Що SFERA вміє з цим апаратом, — див. [PLAN.md](PLAN.md), «Стан роботи». Пам'ятати:

- **Тип апарата.** SFERA читає `MAV_TYPE=1` з HEARTBEAT і не запускає для нього Copter-логіку AUTO/TAKEOFF/RTH.
- **Setpoint-и.** ArduPlane 4.6.3 з `SET_POSITION_TARGET_GLOBAL_INT` бере лише висоту; velocity-only з alt=0 веде апарат у землю
  (FINDINGS §3).
- **Порти.** Стенд SFERA запускає бінарник без MAVProxy на 5760/5762/5763, `sim_vehicle.py` — на тих самих портах: одночасно
  їх не запускати.
