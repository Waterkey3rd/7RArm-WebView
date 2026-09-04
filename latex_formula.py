"""Safe scalar LaTeX formulas for time-parameterized arm trajectories."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import sympy
from sympy.core.function import AppliedUndef
from sympy.parsing.latex import parse_latex


@dataclass(frozen=True)
class Formula:
    latex: str
    expression: sympy.Expr
    _callable: Callable[[float], float]

    @classmethod
    def compile(cls, latex: str) -> "Formula":
        source = latex.strip()
        if source.startswith("$") and source.endswith("$"):
            source = source[1:-1].strip()
        if not source:
            raise ValueError("公式不能为空")
        if len(source) > 500:
            raise ValueError("公式过长")
        try:
            expression = parse_latex(source)
        except Exception as exc:
            raise ValueError(f"LaTeX 公式语法错误: {latex}") from exc
        if not isinstance(expression, sympy.Expr):
            raise ValueError(f"公式不是标量表达式: {latex}")
        # SymPy's ANTLR backend represents LaTeX \pi as a Symbol. Normalize
        # mathematical constants before checking the allowed free variables.
        expression = expression.xreplace({
            sympy.Symbol("pi"): sympy.pi,
            sympy.Symbol("e"): sympy.E,
        })
        t = sympy.Symbol("t")
        unknown = expression.free_symbols - {t}
        if unknown:
            names = ", ".join(sorted(str(symbol) for symbol in unknown))
            raise ValueError(f"公式只允许变量 t，发现: {names}")
        undefined = expression.atoms(AppliedUndef)
        if undefined:
            names = ", ".join(sorted(str(function) for function in undefined))
            raise ValueError(f"不支持的函数: {names}")
        try:
            function = sympy.lambdify(t, expression, modules="math")
        except Exception as exc:
            raise ValueError(f"无法编译公式: {latex}") from exc
        return cls(source, expression, function)

    def evaluate(self, t: float) -> float:
        try:
            value = float(self._callable(float(t)))
        except (ArithmeticError, ValueError, TypeError) as exc:
            raise ValueError(f"公式在 t={t:.6g} 无法计算: {self.latex}") from exc
        if not math.isfinite(value):
            raise ValueError(f"公式在 t={t:.6g} 得到非有限值: {self.latex}")
        return value


if __name__ == "__main__":
    checks = (
        (r"30\sin(2\pi t)", 0.25, 30.0),
        (r"\frac{1}{2}t^2", 2.0, 2.0),
        (r"\sqrt{4}+|-3|", 0.0, 5.0),
    )
    for source, argument, expected in checks:
        actual = Formula.compile(source).evaluate(argument)
        if not math.isclose(actual, expected, abs_tol=1.0e-10):
            raise RuntimeError(f"{source}: expected {expected}, got {actual}")
    print("SymPy LaTeX formula parser OK")
