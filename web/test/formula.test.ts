import { describe, expect, it } from 'vitest';
import { LatexFormula } from '../src/formula';

describe('LaTeX time formulas', () => {
  it('evaluates supported scalar expressions', () => {
    expect(LatexFormula.compile(String.raw`30\sin(2\pi t)`).evaluate(.25)).toBeCloseTo(30, 8);
    expect(LatexFormula.compile(String.raw`\frac{1}{2}t^2`).evaluate(2)).toBeCloseTo(2, 8);
  });

  it('rejects variables other than t', () => {
    expect(() => LatexFormula.compile('a+t')).toThrow(/只允许变量 t/);
  });
});
