import unittest
import tempfile
import shutil
import json
from pathlib import Path
from unittest.mock import patch, MagicMock


from robits.core.config_manager import ConfigManager
from robits.core.config_manager import WorkspaceConfigFinder


class TestWorkspaceConfigFinder(unittest.TestCase):

    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp())
        self.config_file = self.temp_dir / "test_config.json"
        with self.config_file.open("w") as f:
            json.dump({"key": "value"}, f)
        self.finder = WorkspaceConfigFinder(self.temp_dir)

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_find_config(self):
        path = self.finder.find_config("test_config")
        self.assertEqual(path, self.config_file)

    def test_list(self):
        configs = self.finder.list()
        self.assertIn("test_config", configs)

    def test_name_property(self):
        self.assertEqual(self.finder.name, self.temp_dir.name)

    def test_path_property(self):
        self.assertEqual(self.finder.path, self.temp_dir)


class TestConfigManager(unittest.TestCase):

    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp())
        self.config_file = self.temp_dir / "robot_test.json"
        with self.config_file.open("w") as f:
            json.dump({"robot": "test"}, f)

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    @patch.dict("os.environ", {"ROBITS_CONFIG_DIR": str(tempfile.gettempdir())})
    @patch("robits.core.config_manager.PackageResourceFinder", autospec=True)
    def test_get_config_path_and_load_dict(self, mock_package_finder_class):
        mock_finder = MagicMock()
        mock_finder.find_config.return_value = self.config_file
        mock_finder.list.return_value = ["robot_test"]

        mock_package_finder_class.return_value = mock_finder

        with patch(
            "robits.core.config_manager.WorkspaceConfigFinder"
        ) as mock_workspace_finder_class:
            mock_workspace_finder = MagicMock()
            mock_workspace_finder.find_config.return_value = self.config_file
            mock_workspace_finder.list.return_value = ["robot_test"]
            mock_workspace_finder.path = Path(tempfile.gettempdir())
            mock_workspace_finder.name = "mock"

            mock_workspace_finder_class.return_value = mock_workspace_finder

            config_manager = ConfigManager()
            path = config_manager.get_config_path("robot_test")
            self.assertEqual(path, self.config_file)

            data = config_manager.load_dict("robot_test")
            self.assertEqual(data, {"robot": "test"})

            configs = config_manager.list()
            self.assertIn("robot_test", configs)


if __name__ == "__main__":
    unittest.main()
