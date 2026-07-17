from pathlib import Path
import unittest


class YoloV8DependencyContractTest(unittest.TestCase):
    def setUp(self):
        package_root = Path(__file__).resolve().parents[1]
        self.package_xml = (package_root / "package.xml").read_text()
        self.setup_py = (package_root / "setup.py").read_text()
        self.readme = (package_root / "README.md").read_text()
        self.production_sources = "\n".join(
            path.read_text()
            for path in (package_root / "hyu_perception").glob(
                "*.py"
            )
        )

    def test_ros_manifest_documents_pip_runtime_boundary(self):
        self.assertIn(
            "<exec_depend>python3-pip</exec_depend>",
            self.package_xml,
        )
        self.assertIn(
            "Ultralytics has no Galactic rosdep key",
            self.package_xml,
        )

    def test_ultralytics_is_authoritative_python_dependency(self):
        self.assertIn('"ultralytics>=8.4.17,<9"', self.setup_py)
        self.assertIn("authoritative PyPI runtime dependency", self.readme)

    def test_runtime_does_not_depend_on_cv_bridge(self):
        self.assertNotIn("<exec_depend>cv_bridge</exec_depend>", self.package_xml)
        self.assertNotIn("import cv_bridge", self.production_sources)
        self.assertNotIn("from cv_bridge", self.production_sources)
