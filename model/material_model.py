from __future__ import annotations

"""
cleanser.py

将 6 类垃圾组分输入清洗并翻译为优化器所需的等效性质。

输入顺序固定为：
[菜叶, 西瓜皮, 橙子皮, 肉, 杂项混合, 米饭]

输出：
- 等效入口初始含水率 omega0
- 参考干燥时间 tref_min
- 温度敏感性 slope_min_per_c

依赖
----
pip install numpy
"""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence
import numpy as np

from model.model_types import Config, EquivalentProperties

NAMES = ["菜叶", "西瓜皮", "橙子皮", "肉", "杂项混合", "米饭"]
OMEGA = np.array([0.948, 0.948, 0.817, 0.442, 0.773, 0.611], dtype=float)
TREF_TIME = np.array([12.1, 17.7, 15.3, 11.5, 16.3, 15.8], dtype=float)
SLOPE = np.array([-0.132, -0.251, -0.189, -0.216, -0.210, -0.243], dtype=float)


@dataclass(frozen=True)
class CompositionInput:
    values: np.ndarray

    def as_dict(self) -> Dict[str, float]:
        return {name: float(v) for name, v in zip(NAMES, self.values)}


@dataclass(frozen=True)
class CleansedComposition:
    composition: CompositionInput
    equivalent: EquivalentProperties
    ceq_kJ_per_kgK: float



def validate_composition(values: Sequence[float], *, normalize: bool = False) -> CompositionInput:
    arr = np.array(values, dtype=float).reshape(-1)
    if arr.size != 6:
        raise ValueError("组分输入必须正好包含 6 项，对应 [菜叶, 西瓜皮, 橙子皮, 肉, 杂项混合, 米饭]。")
    if np.any(arr < 0.0):
        raise ValueError("组分比例不能为负数。")
    total = float(arr.sum())
    if normalize:
        if total <= 0.0:
            raise ValueError("当 normalize=True 时，组分和必须为正数。")
        arr = arr / total
    else:
        if not np.isclose(total, 1.0, atol=1e-8):
            raise ValueError(f"组分比例和为 {total:.10f}，不等于 1。")
    return CompositionInput(values=arr)



def composition_to_equivalent_properties(
    values: Sequence[float],
    *,
    normalize: bool = False,
    cfg: Config | None = None,
) -> CleansedComposition:
    cfg = cfg or Config()
    comp = validate_composition(values, normalize=normalize)
    x = comp.values
    omega0 = float(np.dot(x, OMEGA))
    tref = float(np.dot(x, TREF_TIME))
    slope = float(np.dot(x, SLOPE))
    ceq = (1.0 - omega0) * cfg.CS + omega0 * cfg.CW
    eq = EquivalentProperties(
        omega0=omega0,
        tref_min=tref,
        slope_min_per_c=slope,
    )
    return CleansedComposition(
        composition=comp,
        equivalent=eq,
        ceq_kJ_per_kgK=float(ceq),
    )



def batch_compositions_to_equivalent_properties(
    rows: Iterable[Sequence[float]],
    *,
    normalize: bool = False,
    cfg: Config | None = None,
) -> List[CleansedComposition]:
    cfg = cfg or Config()
    return [
        composition_to_equivalent_properties(row, normalize=normalize, cfg=cfg)
        for row in rows
    ]



def print_cleansed_result(result: CleansedComposition) -> None:
    print("=" * 72)
    print("一、清洗后的组分比例")
    for name, value in result.composition.as_dict().items():
        print(f"{name:<8s}: {value:.6f}")
    print(f"比例和: {result.composition.values.sum():.6f}")

    print("\n" + "=" * 72)
    print("二、等效性质")
    print(f"omega0            = {result.equivalent.omega0:.6f} ({result.equivalent.omega0*100:.2f}%)")
    print(f"tref_min          = {result.equivalent.tref_min:.6f} min")
    print(f"slope_min_per_c   = {result.equivalent.slope_min_per_c:.6f} min/°C")
    print(f"ceq_kJ_per_kgK    = {result.ceq_kJ_per_kgK:.6f}")


__all__ = [
    "NAMES",
    "CompositionInput",
    "CleansedComposition",
    "validate_composition",
    "composition_to_equivalent_properties",
    "batch_compositions_to_equivalent_properties",
    "print_cleansed_result",
]
