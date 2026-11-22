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
