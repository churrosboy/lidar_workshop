import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gl5_detection.gl5_obstacle_node import read_region, write_region


class RegionStorageTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'region.json'
        self.region = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)]

    def test_existing_version_one_file_and_empty_region(self):
        self.path.write_text(json.dumps({
            'version': 1, 'frame_id': 'laser', 'vertices': [[0, 0], [2, 0], [0, 2]],
        }))
        self.assertEqual(read_region(self.path, 'laser'), self.region)
        write_region(self.path, 'laser', [])
        self.assertEqual(read_region(self.path, 'laser'), [])

    def test_replace_failure_preserves_previous_file_and_removes_temporary_file(self):
        write_region(self.path, 'laser', self.region)
        previous = self.path.read_bytes()
        with patch('gl5_detection.gl5_obstacle_node.os.replace', side_effect=OSError('disk unavailable')):
            with self.assertRaises(OSError):
                write_region(self.path, 'laser', [])
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])

    def test_rejects_wrong_frame_version_and_invalid_polygon(self):
        invalid_files = [
            {'version': 2, 'frame_id': 'laser', 'vertices': self.region},
            {'version': 1, 'frame_id': 'map', 'vertices': self.region},
            {'version': 1, 'frame_id': 'laser', 'vertices': [[0, 0], [1, 1], [2, 2]]},
        ]
        for data in invalid_files:
            with self.subTest(data=data):
                self.path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    read_region(self.path, 'laser')


if __name__ == '__main__':
    unittest.main()
