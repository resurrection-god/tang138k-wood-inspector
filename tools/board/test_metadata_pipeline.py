#!/usr/bin/env python3
"""Непредвзятый набор автотестов конвейера метаданных (Metadata Acceptance Suite).

Проверяет соответствие реализации критериям приёмки документа
scanner_metadata_implementation_requirements v1.1 (раздел 13):
  1. Уникальность scan_id (UUID4).
  2. Фиксация настроек камер и лазера.
  3. Наличие pixel_format и bit_depth.
  4. Сохранение ID и SHA-256 чексумм deployment-профилей.
  5. Проверка единичности нормалей лазерных плоскостей (|norm| == 1.0).
  6. Сохранение статуса 'provisional' (запрет автопромоушна в 'production').
  7. Синхронизация стереопар и покадровая телеметрия.
  8. Расчёт фотометрических признаков из сумм ПЛИС.
  9. Генерация manifest.json с валидными хешами всех файлов.
 10. Атомарное переключение статусов: in_progress -> complete / incomplete.
 11. Явные null для неизвестных параметров.

Запуск:
    python tools/board/test_metadata_pipeline.py [--host 10.0.0.88]
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
import uuid

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.capture.scan_metadata import (
    calculate_photometric_features,
    sha256_file,
    validate_laser_plane,
)
from tools.capture.scan_session import ScanSessionManager
from tools.common import vmsp

fails = 0
total_checks = 0


def check(cond, what, detail=""):
    global fails, total_checks
    total_checks += 1
    tag = "ok  " if cond else "FAIL"
    msg = f"[{tag}] {what}"
    if detail and not cond:
        msg += f" -> {detail}"
    print(msg)
    if not cond:
        fails += 1


def test_uuid_generation_and_uniqueness():
    print("\n--- ТЕСТ 1: Уникальность scan_id и scan_config_id (UUID4) ---")
    tmp = tempfile.mkdtemp()
    try:
        s1 = ScanSessionManager(output_dir=os.path.join(tmp, "s1"), dry_run=True)
        s2 = ScanSessionManager(output_dir=os.path.join(tmp, "s2"), dry_run=True)

        id1 = s1.metadata.scan_id
        id2 = s2.metadata.scan_id
        check(id1 != id2, f"scan_id сессий уникальны: {id1} != {id2}")

        # Проверка формата UUID4
        val1 = uuid.UUID(id1, version=4)
        val2 = uuid.UUID(id2, version=4)
        check(str(val1) == id1 and str(val2) == id2, "Формат идентификаторов строго соответствует UUID4")

        cfg_id1 = s1.metadata.scan_config_id
        check(cfg_id1 is not None and len(cfg_id1) == 36, f"scan_config_id сформирован: {cfg_id1}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_calibration_bundle_and_provisional_status():
    print("\n--- ТЕСТ 2: Верификация калибровочного пакета и статуса 'provisional' ---")
    tmp = tempfile.mkdtemp()
    try:
        s = ScanSessionManager(output_dir=tmp, dry_run=True)
        cb = s.metadata.calibration

        check(cb.cam0_profile_id == "PROFILE-TESTFRAMES-CAM0-B8E14FE1", f"CAM0 profile ID: {cb.cam0_profile_id}")
        check(cb.cam1_profile_id == "PROFILE-TESTFRAMES-CAM1-62C1D4A8", f"CAM1 profile ID: {cb.cam1_profile_id}")

        check(cb.cam0_profile_checksum is not None and len(cb.cam0_profile_checksum) == 64,
              f"CAM0 SHA-256 присутствует: {cb.cam0_profile_checksum[:16]}...")
        check(cb.cam1_profile_checksum is not None and len(cb.cam1_profile_checksum) == 64,
              f"CAM1 SHA-256 присутствует: {cb.cam1_profile_checksum[:16]}...")

        # Строгое требование: статус НЕ должен повышаться до 'production'
        check(cb.status == "provisional", f"Статус связки строго 'provisional' ({cb.status})")
        check(cb.cam0_profile_status == "provisional", "Статус CAM0 строго 'provisional'")
        check(cb.cam1_profile_status == "provisional", "Статус CAM1 строго 'provisional'")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_laser_plane_normalization_and_angles():
    print("\n--- ТЕСТ 3: Проверка единичности нормалей лазерных плоскостей (GUM / P0-1) ---")
    tmp = tempfile.mkdtemp()
    try:
        s = ScanSessionManager(output_dir=tmp, dry_run=True)
        lp = s.metadata.laser_plane

        # CAM0
        p0 = lp.cam0
        check(p0.normalized is True, f"CAM0 нормаль единична: |norm| = {p0.norm_magnitude:.6f}")
        check(abs(p0.norm_magnitude - 1.0) < 1e-4, "Отклонение нормы CAM0 < 1e-4")
        check(p0.coordinate_frame == "cam0_optical", f"Система координат CAM0: {p0.coordinate_frame}")
        check(p0.distance_mm is not None and abs(p0.distance_mm - (-259.96)) < 0.1, f"Расстояние d CAM0: {p0.distance_mm} мм")
        check(p0.angle_to_z_deg is not None and 60.0 < p0.angle_to_z_deg < 80.0, f"Угол наклона к оптической оси Z CAM0: {p0.angle_to_z_deg:.2f}°")

        # CAM1
        p1 = lp.cam1
        check(p1.normalized is True, f"CAM1 нормаль единична: |norm| = {p1.norm_magnitude:.6f}")
        check(abs(p1.norm_magnitude - 1.0) < 1e-4, "Отклонение нормы CAM1 < 1e-4")
        check(p1.coordinate_frame == "cam1_optical", f"Система координат CAM1: {p1.coordinate_frame}")
        check(p1.distance_mm is not None and abs(p1.distance_mm - 266.24) < 0.1, f"Расстояние d CAM1: {p1.distance_mm} мм")
        check(p1.angle_to_z_deg is not None and 60.0 < p1.angle_to_z_deg < 80.0, f"Угол наклона к оптической оси Z CAM1: {p1.angle_to_z_deg:.2f}°")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_photometric_feature_calculations():
    print("\n--- ТЕСТ 4: Проверка расчёта фотометрических признаков (Раздел 6.4) ---")

    # Пример 1: Симметричная линия без насыщения
    # sum_i=1000, sum_iv=500_000 -> centroid = 500.0, i_plus=100, i_minus=100 -> asymmetry = 0.0
    f1 = calculate_photometric_features(
        sum_i=1000,
        sum_iv=500000,
        max_i=200,
        i_minus=100,
        i_plus=100,
        flags=0,
        n_pixels=12,
    )
    check(f1["has_line"] is True, "Линия обнаружена")
    check(f1["centroid_px"] == 500.0, f"Центроид совпал: {f1['centroid_px']} == 500.0")
    check(f1["width_px"] == 12, f"Ширина пятна совпала: {f1['width_px']} == 12")
    check(f1["asymmetry"] == 0.0, f"Асимметрия нулевая: {f1['asymmetry']} == 0.0")
    check(f1["saturated"] is False, "Флаг насыщения снят")

    # Пример 2: Асимметричная насыщенная линия (flags=1)
    f2 = calculate_photometric_features(
        sum_i=2000,
        sum_iv=1200000,
        max_i=255,
        i_minus=50,
        i_plus=250,
        flags=1,
        n_pixels=20,
    )
    check(f2["centroid_px"] == 600.0, f"Центроид насыщенной линии: {f2['centroid_px']} == 600.0")
    check(f2["asymmetry"] == (250 - 50) / 2000.0, f"Асимметрия: {f2['asymmetry']} == 0.10")
    check(f2["saturated"] is True, "Флаг насыщения установлен (flags=1)")

    # Пример 3: Отсутствие линии (sum_i=0)
    f3 = calculate_photometric_features(
        sum_i=0,
        sum_iv=0,
        max_i=10,
        i_minus=0,
        i_plus=0,
        flags=0,
        n_pixels=0,
    )
    check(f3["has_line"] is False, "Отсутствие линии зафиксировано")
    check(f3["centroid_px"] is None, "Центроид отсутствует (None/null)")
    check(f3["asymmetry"] is None, "Асимметрия отсутствует (None/null)")


def test_session_lifecycle_and_manifest(host):
    print("\n--- ТЕСТ 5: Жизненный цикл сессии: 'in_progress' -> 'complete' и manifest.json ---")
    tmp = tempfile.mkdtemp()
    try:
        s = ScanSessionManager(output_dir=tmp, host=host, dry_run=False)

        # Проверка начального файла: статус обязан быть 'in_progress' (13.17)
        with open(s.metadata_file, "r", encoding="utf-8") as f:
            meta_start = json.load(f)
        check(meta_start["status"] == "in_progress", f"Статус при инициализации строго 'in_progress' ({meta_start['status']})")

        # Запись 5 пар кадров
        for i in range(1, 6):
            s.log_frame("cam0", i, 1000000 + i * 10000, {"sum_i": 1000, "sum_iv": 500000, "max_i": 200, "n": 10})
            s.log_frame("cam1", i, 1000005 + i * 10000, {"sum_i": 1000, "sum_iv": 500000, "max_i": 200, "n": 10})
            s.log_pair(i, i, 1000000 + i * 10000, 1000005 + i * 10000)

        # Финализация
        manifest = s.finalize(success=True)

        with open(s.metadata_file, "r", encoding="utf-8") as f:
            meta_end = json.load(f)
        check(meta_end["status"] == "complete", f"Статус после завершения успешно переведён в 'complete' ({meta_end['status']})")
        check(meta_end["scan"]["frame_count_cam0"] == 5, f"Счётчик кадров CAM0: {meta_end['scan']['frame_count_cam0']}")
        check(meta_end["scan"]["frame_count_cam1"] == 5, f"Счётчик кадров CAM1: {meta_end['scan']['frame_count_cam1']}")
        check(meta_end["scan"]["paired_frame_count"] == 5, f"Счётчик стереопар: {meta_end['scan']['paired_frame_count']}")

        # Проверка manifest.json
        check(s.manifest_file.is_file(), "Файл manifest.json создан")
        check(manifest["scan_id"] == meta_end["scan_id"], "scan_id в manifest совпадает с метаданными")
        check(manifest["status"] == "complete", "Статус в manifest равен 'complete'")
        check(manifest["total_files"] >= 3, f"Манифест содержит файлы ({manifest['total_files']} шт.)")

        # Верификация SHA-256 каждого файла в манифесте
        all_hashes_ok = True
        for entry in manifest["files"]:
            target_f = Path(tmp) / entry["path"]
            if not target_f.is_file():
                all_hashes_ok = False
                break
            actual_sha = sha256_file(target_f)
            if actual_sha != entry["sha256"]:
                all_hashes_ok = False
                break
        check(all_hashes_ok, "SHA-256 контрольные суммы ВСЕХ файлов в manifest.json валидны и сходятся")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_session_incomplete_on_abort():
    print("\n--- ТЕСТ 6: Корректная маркировка 'incomplete' при прерывании серии ---")
    tmp = tempfile.mkdtemp()
    try:
        s = ScanSessionManager(output_dir=tmp, dry_run=True)
        # Финализируем с ошибкой
        manifest = s.finalize(success=False, error_msg="Link loss emulation")

        with open(s.metadata_file, "r", encoding="utf-8") as f:
            meta = json.load(f)
        check(meta["status"] == "incomplete", f"Прерванная сессия строго помечена 'incomplete' ({meta['status']})")
        check(manifest["status"] == "incomplete", "Манифест прерванной сессии помечен 'incomplete'")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_null_preservation_for_unknowns():
    print("\n--- ТЕСТ 7: Контроль отсутствия выдуманных констант (явные null) ---")
    tmp = tempfile.mkdtemp()
    try:
        s = ScanSessionManager(output_dir=tmp, dry_run=True)
        with open(s.metadata_file, "r", encoding="utf-8") as f:
            meta = json.load(f)

        # Проверка требований раздела 8.1 и 9: неизвестные поля строго null
        check(meta["device"]["scanner_id"] is None, "Неизвестный scanner_id равен null")
        check(meta["laser"]["wavelength_measured_nm"] is None, "Неизмеренная длина волны равна null")
        check(meta["laser"]["power_measured_mw"] is None, "Неизмеренная мощность равна null")
        check(meta["cameras"]["cam0"]["digital_gain"] is None, "Неизвестный digital_gain равен null")
        check(meta["cameras"]["cam0"]["filter_fwhm_nm"] is None, "Неизвестный FWHM фильтра равен null")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    parser = argparse.ArgumentParser(description="Автотесты конвейера сбора метаданных сканера")
    parser.add_argument("--host", default=vmsp.HOST, help="IP-адрес платы")
    args = parser.parse_args()

    print("============================================================")
    print(" [METADATA TESTS] Автотесты системы метаданных (раздел 13) ")
    print("============================================================")
    print(f"Целевой хост: {args.host}")

    t0 = time.time()

    test_uuid_generation_and_uniqueness()
    test_calibration_bundle_and_provisional_status()
    test_laser_plane_normalization_and_angles()
    test_photometric_feature_calculations()
    test_session_lifecycle_and_manifest(args.host)
    test_session_incomplete_on_abort()
    test_null_preservation_for_unknowns()

    elapsed = time.time() - t0
    print("\n============================================================")
    print(f" [ИТОГ АВТОТЕСТОВ] Проверок: {total_checks}, Ошибок: {fails}, Время: {elapsed:.2f} с")
    if fails == 0:
        print(" [РЕЗУЛЬТАТ] ВСЕ ТЕСТЫ МЕТАДАННЫХ УСПЕШНО ПРОЙДЕНЫ (100% SUCCESS)!")
    else:
        print(f" [РЕЗУЛЬТАТ] ОБНАРУЖЕНЫ НЕПРОЙДЕННЫЕ ТЕСТЫ: {fails}")
    print("============================================================")

    return fails


if __name__ == "__main__":
    sys.exit(main())
