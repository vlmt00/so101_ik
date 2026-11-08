"""Load the SO-101 URDF and attach a fixed TCP to its moving jaw."""
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
import numpy as np

@dataclass(frozen=True)
class TCPConfig:
    parent_link: str = "moving_jaw_so101_v1_link"
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    link_name: str = "tcp_link"
    joint_name: str = "tcp_joint"
    tip_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)

    def __post_init__(self):
        for vec in (self.xyz, self.rpy, self.tip_axis):
            if len(vec) != 3 or not np.all(np.isfinite(vec)):
                raise ValueError("TCP vectors must each contain 3 finite numbers")
        if np.linalg.norm(self.tip_axis) < 1e-12:
            raise ValueError("tip_axis must not be zero")

class RobotModel:
    def __init__(self, urdf_path: str | Path, tcp: TCPConfig | None = None):
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(self.urdf_path)
        self.tcp = tcp or TCPConfig()
        self._temp = TemporaryDirectory(prefix="so101_tcp_")
        try:
            self._generated_urdf = self._generate_urdf()
            try:
                from roboticstoolbox import ERobot
            except ImportError as exc:
                raise ImportError("Install roboticstoolbox-python; see README") from exc
            self.robot = ERobot.URDF(str(self._generated_urdf))
            self.n = int(self.robot.n)
            self.qlim = np.asarray(self.robot.qlim, dtype=float)
            if self.n != 6 or self.qlim.shape != (2, self.n):
                raise ValueError("Expected a 6-actuated-joint SO-101 model")
            self.joint_names = tuple(link.jname for link in self.robot.links
                                     if getattr(link, "isjoint", False))
        except Exception:
            self.close()
            raise

    @staticmethod
    def _vector(values):
        return " ".join(f"{float(v):.12g}" for v in values)

    def _generate_urdf(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        links = {x.get("name") for x in root.findall("link")}
        joints = {x.get("name") for x in root.findall("joint")}
        if self.tcp.parent_link not in links:
            raise ValueError(f"Missing jaw link: {self.tcp.parent_link}; found {sorted(links)}")
        if self.tcp.link_name in links or self.tcp.joint_name in joints:
            raise ValueError("TCP link/joint already exists")
        ET.SubElement(root, "link", {"name": self.tcp.link_name})
        joint = ET.SubElement(root, "joint", {"name": self.tcp.joint_name, "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": self.tcp.parent_link})
        ET.SubElement(joint, "child", {"link": self.tcp.link_name})
        ET.SubElement(joint, "origin", {
            "xyz": self._vector(self.tcp.xyz),
            "rpy": self._vector(self.tcp.rpy)})
        output_dir = Path(self._temp.name)
        # Keep original relative mesh filenames valid in temporary URDF.
        for mesh_element in root.iter("mesh"):
            raw = mesh_element.get("filename", "")
            if raw.startswith("package://"):
                raise ValueError("Resolve package:// mesh paths to relative paths first")
            mesh = Path(raw)
            if mesh.is_absolute():
                continue
            original = (self.urdf_path.parent / mesh).resolve()
            if not original.is_file():
                raise FileNotFoundError(f"Missing mesh {raw}: {original}")
            target = output_dir / mesh
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                target.symlink_to(original)
        output = output_dir / self.urdf_path.name
        tree.write(output, encoding="utf-8", xml_declaration=True)
        return output

    def close(self):
        self._temp.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
