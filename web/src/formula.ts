import { ComputeEngine, type BoxedExpression } from '@cortex-js/compute-engine';

export class LatexFormula {
  private static readonly engine = new ComputeEngine();
  private constructor(readonly source: string, private readonly expression: BoxedExpression) {}

  static compile(latex: string): LatexFormula {
    const source = latex.trim().replace(/^\$(.*)\$$/s, '$1').trim();
    if (!source) throw new Error('公式不能为空');
    if (source.length > 500) throw new Error('公式长度不能超过 500 字符');
    const expression = this.engine.parse(source);
    if (expression.errors.length) throw new Error(`LaTeX 公式语法错误：${source}`);
    const unknown = expression.symbols.filter(symbol => !['t', 'Pi', 'ExponentialE', 'ImaginaryUnit'].includes(symbol));
    if (unknown.length) throw new Error(`公式只允许变量 t，发现：${unknown.join(', ')}`);
    return new LatexFormula(source, expression);
  }

  evaluate(t: number): number {
    const numeric = this.expression.subs({ t }).N().numericValue;
    const value = typeof numeric === 'number' ? numeric : (numeric && numeric.im === 0 ? numeric.re : Number.NaN);
    if (!Number.isFinite(value)) throw new Error(`公式在 t=${t} 无法得到有限实数：${this.source}`);
    return value;
  }
}
