"""Package the Windows launcher and canonical protocol without copying ROS."""
from pathlib import Path
import zipfile

here = Path(__file__).resolve().parent
root = here.parents[1]
output = here / 'k7-espnow-windows-test.zip'
with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
    for name in ('run.sh', 'run.cmd', 'windows_test.py', 'README.md'):
        archive.write(here / name, 'k7-espnow-windows-test/' + name)
    archive.write(root / 'ROS2_src/K7_ros2/src/k7_tasks/k7_tasks/espnow_protocol.py',
                  'k7-espnow-windows-test/espnow_protocol.py')
print(output)
