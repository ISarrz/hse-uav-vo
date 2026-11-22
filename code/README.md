# Код курсовой

Окружение для сравнения ORB-SLAM3, VINS-Fusion и OpenVINS. Каждая система собирается в отдельном контейнере podman (или docker), версии зафиксированы в `containers/*.Containerfile`.

| Образ | Система | База |
|---|---|---|
| `localhost/uav-orbslam3` | ORB-SLAM3 (master, `4452a3c`) + Pangolin v0.6 | Ubuntu 20.04, ROS Noetic |
| `localhost/uav-vins` | VINS-Fusion (master, `be55a93`), патч под OpenCV 4 | Ubuntu 20.04, ROS Noetic |
| `localhost/uav-openvins` | OpenVINS v2.7 | Ubuntu 22.04, ROS 2 Humble |

## Сборка

```bash
scripts/build_images.sh                 # все три образа
scripts/build_images.sh openvins        # один образ
```

Логи сборки пишутся в `logs/`. Сборка идёт с `--network=host`: в rootless podman сеть `pasta` падает, если в системе много маршрутов (например, при включённом VPN).

## Метрики

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/evo_ape tum groundtruth.txt estimate.txt -a
```

## Данные EuRoC

```bash
python3 -I scripts/fetch_euroc.py --list
python3 -I scripts/fetch_euroc.py MH_01_easy MH_03_medium MH_05_difficult V1_02_medium V2_02_medium
.venv/bin/python -I scripts/euroc_to_bag.py data/euroc/MH_01_easy --ros1 --ros2
```

## Запуск и оценка

```bash
scripts/run_orbslam3.sh MH_01_easy mono_inertial
scripts/run_vins.sh MH_01_easy mono_imu
scripts/run_openvins.sh MH_01_easy mono
.venv/bin/python -I scripts/evaluate.py MH_01_easy
```

Траектории (формат TUM), логи и графики сохраняются в `results/<последовательность>/`, сводная таблица -- в `results/summary.csv`.

Кроме итоговой траектории сохраняются **онлайн-оценки** -- поза, которую система выдала в момент кадра, без последующих уточнений:

| Файл | Что это |
|---|---|
| `orbslam3_<режим>.tum` | итоговая траектория ORB-SLAM3 (после петель и глобальной оптимизации) |
| `orbslam3_<режим>_online.tum` | онлайн-оценка ORB-SLAM3 (патч `containers/patches/orbslam3_online_pose.py`) |
| `vins_<режим>.tum` | VIO VINS-Fusion (пишется по ходу работы) |
| `vins_<режим>_loop.tum` | траектория после замыкания петель (итоговая) |
| `vins_<режим>_loop_online.tum` | онлайн-оценка с учётом петель (топик `odometry_rect`) |
| `openvins_<режим>.tum` | OpenVINS (пишется по ходу работы) |

## Повторные запуски

```bash
scripts/run_repeated.sh vins MH_01_easy mono_imu 5     # -> results/MH_01_easy/runs/*_run<k>.tum
.venv/bin/python -I scripts/evaluate.py MH_01_easy --runs
```

Выводятся среднее и разброс ATE и дрейфа по удачным запускам и число разошедшихся (ATE > 1 м).

## Метрика потери GNSS

```bash
.venv/bin/python -I scripts/evaluate.py MH_01_easy --gnss-loss [--runs] [--window 10] [--t0-step 5]
```

Моделирует отказ GNSS в момент t0 без перезапуска систем: оценка совмещается с эталоном только по окну `[t0 - window, t0]` (пока GNSS работает, автопилот знает положение и курс), дальше ошибка положения считается без повторного совмещения. t0 перебирается с шагом `--t0-step` по всему полёту, результат -- ошибка в зависимости от времени и пути после t0 (медиана и 90-й перцентиль по всем t0).

- Считается только по онлайн-оценкам: итоговые траектории уточнены с учётом будущих кадров.
- Онлайн-траектория делится на непрерывные участки: граница -- пауза в выдаче поз дольше 0,25 с или скачок со скоростью больше 5 м/с (сброс карты, инерциальная инициализация, слияние карт). Окно до t0 и продолжение после t0 берутся с одного участка: после скачка навигация по этой оценке считается потерянной.
- Выход: `results/<посл.>/gnss_loss.png`, `gnss_loss_curves.csv`, сводка `results/gnss_loss_summary.csv` (ошибка через 10/30/60 с и 10/25/50 м).
