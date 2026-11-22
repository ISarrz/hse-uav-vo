"""Запись онлайн-оценки позы в примерах ORB-SLAM3 для EuRoC.

Штатно пример сохраняет траекторию в конце прогона, после всех замыканий петель
и глобальной оптимизации. Для моделирования потери GNSS нужна поза, которую
система выдала в момент кадра, -- её пишем в online_trajectory.txt:
    t[с] состояние_трекинга tx ty tz qx qy qz qw   (поза камеры Twc)
"""
import re
import sys

WRITE = r"""
            {
                static std::ofstream f_online("online_trajectory.txt");
                Sophus::SE3f Twc_online = Tcw_online.inverse();
                Eigen::Vector3f p = Twc_online.translation();
                Eigen::Quaternionf q = Twc_online.unit_quaternion();
                f_online << std::fixed << std::setprecision(9) << tframe << " " << SLAM.GetTrackingState()
                         << " " << p.x() << " " << p.y() << " " << p.z()
                         << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
                f_online.flush();
            }
"""

for path in sys.argv[1:]:
    src = open(path).read()
    if "online_trajectory.txt" in src:
        continue
    if "#include<iomanip>" not in src and "#include <iomanip>" not in src:
        src = src.replace("#include<fstream>", "#include<fstream>\n#include<iomanip>", 1)
    src, n = re.subn(r"^(\s*)(SLAM\.Track(?:Monocular|Stereo)\([^;]*\);)",
                     lambda m: f"{m.group(1)}Sophus::SE3f Tcw_online = {m.group(2)}{WRITE}",
                     src, flags=re.M)
    if n == 0:
        sys.exit(f"{path}: вызов TrackMonocular/TrackStereo не найден")
    open(path, "w").write(src)
    print(f"{path}: патч применён ({n})")
