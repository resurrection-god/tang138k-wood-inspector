"""Общее для инструментов ПК: протокол платы (vmsp) и карта её памяти (memory_map).

Инструменты подключают пакет так (из tools/<группа>/x.py):

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    from common import memory_map as mm
"""
