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

  it('can compile a batch without losing the parser engine context', () => {
    const formulas = ['1', String.raw`25\cos(2\pi t)`].map(LatexFormula.compile);
    expect(formulas[1].evaluate(0)).toBeCloseTo(25, 8);
  });
});
