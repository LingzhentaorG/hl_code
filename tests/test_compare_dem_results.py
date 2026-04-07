#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

import numpy as np
import rasterio
from rasterio.transform import from_origin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from compare_dem_results import compare_dem_results


def write_raster(path: Path, array: np.ndarray, transform, nodata: float, crs: str = "EPSG:4326") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=array.shape[0],
        width=array.shape[1],
        count=1,
        dtype=array.dtype,
        transform=transform,
        nodata=nodata,
        crs=crs,
        compress="deflate",
    ) as dataset:
        dataset.write(array, 1)


def write_mask(path: Path, mask: np.ndarray, transform, crs: str = "EPSG:4326") -> None:
    write_raster(path, mask.astype(np.float32), transform, nodata=0.0, crs=crs)


def plane_from_transform(transform, rows: int, cols: int) -> np.ndarray:
    xs = float(transform.c) + (np.arange(cols, dtype=np.float64) + 0.5) * float(transform.a)
    ys = float(transform.f) - (np.arange(rows, dtype=np.float64) + 0.5) * abs(float(transform.e))
    xx, yy = np.meshgrid(xs, ys)
    return (xx + 2.0 * yy).astype(np.float64)


class CompareDemResultsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = Path(tempfile.mkdtemp(prefix="dem_compare_test_"))

    def tearDown(self) -> None:
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def _build_main_mask_dir(
        self,
        transform,
        shape: tuple[int, int],
        domain: np.ndarray | None = None,
        object_mask: np.ndarray | None = None,
        boundary_mask: np.ndarray | None = None,
    ) -> Path:
        mask_dir = self.temp_dir / "mask"
        default = np.zeros(shape, dtype=np.uint8)
        write_mask(mask_dir / "dem_domain_mask.tif", default if domain is None else domain, transform)
        write_mask(mask_dir / "dem_object_mask.tif", default if object_mask is None else object_mask, transform)
        write_mask(mask_dir / "dem_boundary_mask.tif", default if boundary_mask is None else boundary_mask, transform)
        return mask_dir

    def test_integer_window_alignment_reports_offsets_metrics_and_hotspots(self) -> None:
        main_transform = from_origin(0.0, 6.0, 1.0, 1.0)
        reference_transform = from_origin(1.0, 5.0, 1.0, 1.0)
        nodata = -9999.0

        main = np.full((6, 6), nodata, dtype=np.float64)
        main[1:5, 1:5] = np.arange(16, dtype=np.float64).reshape(4, 4)
        main[2, 2] = nodata

        reference = main[1:5, 1:5].astype(np.float32)
        reference[0, 0] = reference[0, 0] + np.float32(12.0)
        reference[1, 1] = np.float32(22.0)
        reference[3, 3] = np.float32(nodata)

        main_path = self.temp_dir / "main.tif"
        reference_path = self.temp_dir / "reference.tif"
        write_raster(main_path, main, main_transform, nodata)
        write_raster(reference_path, reference, reference_transform, nodata)

        object_mask = np.zeros(main.shape, dtype=np.uint8)
        boundary_mask = np.zeros(main.shape, dtype=np.uint8)
        object_mask[1, 1] = 1
        boundary_mask[1, 1] = 1
        mask_dir = self._build_main_mask_dir(main_transform, main.shape, object_mask=object_mask, boundary_mask=boundary_mask)

        output_dir = self.temp_dir / "comparison"
        report = compare_dem_results(main_path, reference_path, mask_dir, output_dir, dataset_name="synthetic_integer")

        self.assertEqual(report["alignment"]["mode"], "integer_window")
        self.assertEqual(report["alignment"]["row_offset"], 1)
        self.assertEqual(report["alignment"]["col_offset"], 1)
        self.assertAlmostEqual(report["domain"]["jaccard"], 14.0 / 16.0, places=9)
        self.assertEqual(report["domain"]["only_main_count"], 1)
        self.assertEqual(report["domain"]["only_reference_count"], 1)
        self.assertAlmostEqual(report["elevation"]["rmse"], math.sqrt(144.0 / 14.0), places=9)
        self.assertEqual(report["hotspots"]["counts"]["gt_10m"], 1)
        self.assertEqual(report["hotspots"]["top_records"][0]["row"], 1)
        self.assertEqual(report["hotspots"]["top_records"][0]["col"], 1)
        self.assertTrue(report["hotspots"]["top_records"][0]["on_object_mask"])
        self.assertTrue(report["hotspots"]["top_records"][0]["on_boundary_mask"])
        self.assertTrue((output_dir / "report.json").exists())
        self.assertTrue((output_dir / "summary.md").exists())
        self.assertTrue((output_dir / "domain_overlap.png").exists())
        self.assertTrue((output_dir / "diff_signed.png").exists())
        self.assertTrue((output_dir / "diff_abs.png").exists())
        self.assertTrue((output_dir / "slope_diff.png").exists())
        self.assertTrue((output_dir / "hotspots.csv").exists())

    def test_reproject_alignment_uses_bilinear_for_linear_surface(self) -> None:
        main_transform = from_origin(0.0, 4.0, 1.0, 1.0)
        reference_transform = from_origin(-0.5, 4.5, 1.0, 1.0)
        nodata = -9999.0

        main = plane_from_transform(main_transform, 4, 4)
        reference = plane_from_transform(reference_transform, 5, 5).astype(np.float32)

        main_path = self.temp_dir / "main_reproject.tif"
        reference_path = self.temp_dir / "reference_reproject.tif"
        write_raster(main_path, main, main_transform, nodata)
        write_raster(reference_path, reference, reference_transform, nodata)
        mask_dir = self._build_main_mask_dir(main_transform, main.shape)

        report = compare_dem_results(main_path, reference_path, mask_dir, self.temp_dir / "comparison_reproject", dataset_name="synthetic_reproject")

        self.assertEqual(report["alignment"]["mode"], "reprojected")
        self.assertEqual(report["domain"]["main_valid_count"], 16)
        self.assertEqual(report["domain"]["reference_valid_count_aligned"], 16)
        self.assertLess(report["elevation"]["rmse"], 1e-6)
        self.assertLess(report["elevation"]["mae"], 1e-6)
        self.assertEqual(report["hotspots"]["counts"]["gt_10m"], 0)

    def test_batch_process_compare_map_writes_comparison_summaries(self) -> None:
        data_dir = self.temp_dir / "data"
        data_dir.mkdir(parents=True, exist_ok=True)
        (data_dir / "sample.ply").write_text("ply\nformat ascii 1.0\nend_header\n", encoding="utf-8")
        (data_dir / "ignored.ply").write_text("ply\nformat ascii 1.0\nend_header\n", encoding="utf-8")

        config_path = self.temp_dir / "config.json"
        config_path.write_text("{}", encoding="utf-8")

        main_transform = from_origin(0.0, 4.0, 1.0, 1.0)
        reference_transform = from_origin(1.0, 3.0, 1.0, 1.0)
        nodata = -9999.0

        reference = plane_from_transform(reference_transform, 3, 3).astype(np.float32)
        reference_path = self.temp_dir / "reference_batch.tif"
        write_raster(reference_path, reference, reference_transform, nodata)

        fake_cli_py = self.temp_dir / "fake_cli.py"
        fake_cli_py.write_text(
            textwrap.dedent(
                f"""
                import json
                import sys
                from pathlib import Path

                import numpy as np
                import rasterio
                from rasterio.transform import from_origin

                def write(path: Path, array: np.ndarray, transform, nodata: float) -> None:
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with rasterio.open(
                        path,
                        "w",
                        driver="GTiff",
                        height=array.shape[0],
                        width=array.shape[1],
                        count=1,
                        dtype=array.dtype,
                        transform=transform,
                        nodata=nodata,
                        crs="EPSG:4326",
                        compress="deflate",
                    ) as dataset:
                        dataset.write(array, 1)

                output_dir = None
                for index, value in enumerate(sys.argv):
                    if value == "--set" and index + 1 < len(sys.argv):
                        item = sys.argv[index + 1]
                        if item.startswith("output.directory="):
                            output_dir = Path(json.loads(item.split("=", 1)[1]))

                if output_dir is None:
                    raise SystemExit("missing output.directory override")

                nodata = -9999.0
                transform = from_origin(0.0, 4.0, 1.0, 1.0)
                dem = np.array(
                    [
                        [10.0, 11.0, 12.0, 13.0],
                        [14.0, 15.0, 16.0, 17.0],
                        [18.0, 19.0, 20.0, 21.0],
                        [22.0, 23.0, 24.0, 25.0],
                    ],
                    dtype=np.float64,
                )
                write(output_dir / "surface" / "dem.tif", dem, transform, nodata)
                write(output_dir / "surface" / "dem_raw.tif", dem, transform, nodata)
                zeros = np.zeros((4, 4), dtype=np.float32)
                write(output_dir / "mask" / "dem_domain_mask.tif", np.ones((4, 4), dtype=np.float32), transform, 0.0)
                write(output_dir / "mask" / "dem_object_mask.tif", zeros, transform, 0.0)
                write(output_dir / "mask" / "dem_boundary_mask.tif", zeros, transform, 0.0)
                log_dir = output_dir / "log"
                log_dir.mkdir(parents=True, exist_ok=True)
                (log_dir / "stats.txt").write_text("tile_mode_used=0\\n", encoding="utf-8")
                print("ok")
                """
            ).strip()
            + "\n",
            encoding="utf-8",
        )

        if os.name == "nt":
            fake_cli = self.temp_dir / "fake_cli.cmd"
            fake_cli.write_text(f'@echo off\r\n"{sys.executable}" "{fake_cli_py}" %*\r\n', encoding="utf-8")
        else:
            fake_cli = self.temp_dir / "fake_cli.sh"
            fake_cli.write_text(f'#!/bin/sh\nexec "{sys.executable}" "{fake_cli_py}" "$@"\n', encoding="utf-8")
            fake_cli.chmod(fake_cli.stat().st_mode | stat.S_IEXEC)

        compare_map_path = self.temp_dir / "compare_map.json"
        compare_map_path.write_text(json.dumps({"sample": str(reference_path)}, ensure_ascii=False, indent=2), encoding="utf-8")
        manifest_path = self.temp_dir / "manifest.json"
        manifest_path.write_text(json.dumps(["sample.ply"], ensure_ascii=False, indent=2), encoding="utf-8")

        output_root = self.temp_dir / "batch_output"
        script_path = ROOT / "scripts" / "batch_process.py"
        result = subprocess.run(
            [
                sys.executable,
                str(script_path),
                "--cli",
                str(fake_cli),
                "--config",
                str(config_path),
                "--input-dir",
                str(data_dir),
                "--output-root",
                str(output_root),
                "--manifest",
                str(manifest_path),
                "--batch-name",
                "final_release",
                "--compare-map",
                str(compare_map_path),
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr or result.stdout)

        batch_root = output_root / "final_release"
        self.assertTrue(batch_root.exists())
        self.assertTrue((batch_root / "comparison_summary.json").exists())
        self.assertTrue((batch_root / "comparison_summary.csv").exists())
        self.assertTrue((batch_root / "datasets" / "sample" / "comparison" / "report.json").exists())
        self.assertFalse((batch_root / "datasets" / "ignored").exists())
        comparison_records = json.loads((batch_root / "comparison_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(len(comparison_records), 1)
        self.assertEqual(comparison_records[0]["dataset"], "sample")
        self.assertEqual(comparison_records[0]["status"], "success")


RESOURCE3_OUTPUT_CANDIDATES = [
    ROOT / "output" / "final_release" / "datasets" / "资源三号_石林_点云示范",
    ROOT / "output" / "batch_20260407_144339" / "datasets" / "资源三号_石林_点云示范",
]
RESOURCE3_DATASET_DIR = next((path for path in RESOURCE3_OUTPUT_CANDIDATES if path.exists()), RESOURCE3_OUTPUT_CANDIDATES[0])
RESOURCE3_MAIN = RESOURCE3_DATASET_DIR / "surface" / "dem.tif"
RESOURCE3_MASK = RESOURCE3_DATASET_DIR / "mask"
RESOURCE3_REFERENCE = ROOT / "arcgis" / "dem_idw_final.tif"


@unittest.skipUnless(RESOURCE3_MAIN.exists() and RESOURCE3_REFERENCE.exists() and RESOURCE3_MASK.exists(), "resource3 regression assets not available")
class Resource3RegressionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = Path(tempfile.mkdtemp(prefix="dem_compare_resource3_"))

    def tearDown(self) -> None:
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_resource3_regression_metrics(self) -> None:
        report = compare_dem_results(
            main_dem_path=RESOURCE3_MAIN,
            reference_dem_path=RESOURCE3_REFERENCE,
            mask_dir=RESOURCE3_MASK,
            output_dir=self.temp_dir / "comparison",
            dataset_name="资源三号_石林_点云示范",
        )

        self.assertEqual(report["alignment"]["mode"], "integer_window")
        self.assertEqual(report["alignment"]["row_offset"], 7)
        self.assertEqual(report["alignment"]["col_offset"], 1)
        self.assertAlmostEqual(report["domain"]["jaccard"], 0.9974325471352313, places=12)
        self.assertAlmostEqual(report["elevation"]["rmse"], 0.12703927985578292, places=12)
        self.assertAlmostEqual(report["elevation"]["p95_abs_error"], 6.0893929571648187e-05, places=12)
        self.assertAlmostEqual(report["domain"]["only_reference_area"], 2092288.0, places=3)


if __name__ == "__main__":
    unittest.main()
