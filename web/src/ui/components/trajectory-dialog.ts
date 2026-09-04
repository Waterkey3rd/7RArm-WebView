import type { RoboArmController } from '../../controller';
import { LatexFormula } from '../../formula';
import { DEG, RAD } from '../../math';
import { SIDES, type Side, type Space } from '../../types';
import { showToast } from './toast';

export interface TrajectoryPreset {
  name: string;
  space: Space;
  tStart: number;
  tEnd: number;
  durationMs: number;
  keypointCount: number;
  sources: Record<Side, string[]>;
}

export class TrajectoryDialog {
  private backdrop: HTMLElement | null = null;
  private currentSpace: Space = 'JointAngleSpace';
  private formulaInputs: Record<Side, HTMLInputElement[]> = { left: [], right: [] };
  private tStartInput!: HTMLInputElement;
  private tEndInput!: HTMLInputElement;
  private durationInput!: HTMLInputElement;
  private keypointsInput!: HTMLInputElement;
  private allowDiscontinuityCheckbox!: HTMLInputElement;

  constructor(
    private readonly controller: RoboArmController,
    private readonly onGenerated: () => void,
  ) {}

  open(): void {
    if (this.backdrop) return;

    this.backdrop = document.createElement('div');
    this.backdrop.className = 'modal-backdrop';

    this.backdrop.innerHTML = `
      <div class="modal-window" style="max-width: 880px;">
        <div class="modal-header">
          <div class="modal-title">
            <span>ƒ(t)</span>
            <span>添加关于 t 的 LaTeX 函数轨迹</span>
          </div>
          <button class="modal-close-btn" id="modal-close-btn">×</button>
        </div>

        <div class="modal-body">
          <!-- Preset & Space Header -->
          <div style="display: flex; align-items: center; justify-content: space-between; gap: 12px;">
            <div style="display: flex; align-items: center; gap: 8px;">
              <span style="font-size: 13px; font-weight: 700; color: var(--text-secondary);">动作模板预设:</span>
              <select class="form-select" id="select-preset" style="min-width: 220px;">
                <option value="sine">1. 双臂对称正弦波浪 (关节空间)</option>
                <option value="circle">2. 笛卡尔空间圆周绘制 (笛卡尔)</option>
                <option value="helix">3. 空间立体螺旋线 (笛卡尔)</option>
                <option value="wave">4. 协同挥手致意 (关节空间)</option>
              </select>
            </div>

            <div style="display: flex; align-items: center; gap: 8px;">
              <span style="font-size: 13px; font-weight: 700; color: var(--text-secondary);">插值空间:</span>
              <select class="form-select" id="select-space">
                <option value="JointAngleSpace">关节角空间 (deg)</option>
                <option value="CartesianSpace">笛卡尔空间 (mm, deg)</option>
              </select>
            </div>
          </div>

          <!-- Parameters Grid -->
          <div class="form-grid-params">
            <div class="form-field">
              <label class="form-field-label">t 起点 (tStart)</label>
              <input type="number" class="form-input" id="inp-tstart" value="0" step="0.1">
            </div>
            <div class="form-field">
              <label class="form-field-label">t 终点 (tEnd)</label>
              <input type="number" class="form-input" id="inp-tend" value="1" step="0.1">
            </div>
            <div class="form-field">
              <label class="form-field-label">总运行时长 (s)</label>
              <input type="number" class="form-input" id="inp-duration" value="3.0" step="0.5" min="0.3">
            </div>
            <div class="form-field">
              <label class="form-field-label">采样关键点数 N</label>
              <input type="number" class="form-input" id="inp-keypoints" value="20" step="1" min="2" max="300">
            </div>
            <div class="form-field" style="justify-content: flex-end;">
              <label style="display: flex; align-items: center; gap: 6px; font-size: 12px; cursor: pointer; color: var(--text-secondary);">
                <input type="checkbox" id="chk-discontinuity">
                <span>允许首点不连续</span>
              </label>
            </div>
          </div>

          <!-- Alert / Discontinuity Check -->
          <div class="alert-banner info" id="traj-info-banner">
            <span>💡 LaTeX 语法支持 \\sin(2\\pi t)、\\frac{1}{2}t^2、\\cos 等标准数学算式。当前机械臂状态将作为轨迹运动起点。</span>
          </div>

          <!-- Formula Rows for Left and Right Arms -->
          <div class="formula-arms-container" id="formula-arms-host"></div>
        </div>

        <div class="modal-footer">
          <button class="btn btn-secondary" id="btn-snap-current">📐 自动对齐当前姿态</button>
          <button class="btn btn-secondary" id="btn-cancel">取消</button>
          <button class="btn btn-primary" id="btn-generate">
            <span id="gen-spinner" style="display: none;">⏳</span>
            <span>生成并执行轨迹</span>
          </button>
        </div>
      </div>
    `;

    document.body.appendChild(this.backdrop);

    this.tStartInput = this.backdrop.querySelector('#inp-tstart') as HTMLInputElement;
    this.tEndInput = this.backdrop.querySelector('#inp-tend') as HTMLInputElement;
    this.durationInput = this.backdrop.querySelector('#inp-duration') as HTMLInputElement;
    this.keypointsInput = this.backdrop.querySelector('#inp-keypoints') as HTMLInputElement;
    this.allowDiscontinuityCheckbox = this.backdrop.querySelector('#chk-discontinuity') as HTMLInputElement;

    const spaceSelect = this.backdrop.querySelector('#select-space') as HTMLSelectElement;
    spaceSelect.addEventListener('change', () => {
      this.currentSpace = spaceSelect.value as Space;
      this.rebuildFormulaRows();
    });

    const presetSelect = this.backdrop.querySelector('#select-preset') as HTMLSelectElement;
    presetSelect.addEventListener('change', () => {
      this.applyPreset(presetSelect.value);
    });

    this.backdrop.querySelector('#btn-snap-current')?.addEventListener('click', () => {
      this.snapToCurrentArmState();
    });

    this.backdrop.querySelector('#modal-close-btn')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#btn-cancel')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#btn-generate')?.addEventListener('click', () => this.generate());

    this.rebuildFormulaRows();
    this.applyPreset('sine');
  }

  private rebuildFormulaRows(): void {
    const host = this.backdrop?.querySelector('#formula-arms-host');
    if (!host) return;
    host.innerHTML = '';

    const labels = this.currentSpace === 'JointAngleSpace'
      ? ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']
      : ['X', 'Y', 'Z', 'Yaw', 'Pitch', 'Roll'];

    this.formulaInputs = { left: [], right: [] };
    const snapshot = this.controller.snapshotValues();

    for (const side of SIDES) {
      const box = document.createElement('div');
      box.className = 'formula-arm-box';
      const title = side === 'left' ? '左臂公式 f(t)' : '右臂公式 f(t)';
      box.innerHTML = `<div class="formula-arm-title ${side}">${title}</div>`;

      const values = snapshot[side][this.currentSpace];

      labels.forEach((lab, i) => {
        const row = document.createElement('div');
        row.className = 'formula-row';
        row.innerHTML = `
          <span class="formula-var-label">${lab}(t) =</span>
          <input type="text" class="formula-input" id="formula-${side}-${i}" value="${values[i].toFixed(1)}">
        `;
        box.appendChild(row);

        const inp = row.querySelector(`#formula-${side}-${i}`) as HTMLInputElement;
        this.formulaInputs[side].push(inp);
      });

      host.appendChild(box);
    }
  }

  private snapToCurrentArmState(): void {
    const snapshot = this.controller.snapshotValues();
    const tStart = parseFloat(this.tStartInput.value) || 0;

    for (const side of SIDES) {
      const cur = snapshot[side][this.currentSpace];
      this.formulaInputs[side].forEach((inp, i) => {
        try {
          const formula = LatexFormula.compile(inp.value);
          const valAtStart = formula.evaluate(tStart);
          const diff = cur[i] - valAtStart;
          if (Math.abs(diff) > 1e-4) {
            const sign = diff >= 0 ? `+ ${diff.toFixed(2)}` : `- ${Math.abs(diff).toFixed(2)}`;
            inp.value = `(${inp.value}) ${sign}`;
          }
        } catch {
          inp.value = cur[i].toFixed(2);
        }
      });
    }
    showToast({ message: '已自动修正各轴初值与机械臂当前姿态对齐', type: 'success' });
  }

  private applyPreset(key: string): void {
    const spaceSelect = this.backdrop?.querySelector('#select-space') as HTMLSelectElement;

    if (key === 'sine') {
      this.currentSpace = 'JointAngleSpace';
      if (spaceSelect) spaceSelect.value = 'JointAngleSpace';
      this.rebuildFormulaRows();
      this.tStartInput.value = '0';
      this.tEndInput.value = '1';
      this.durationInput.value = '3.0';
      this.keypointsInput.value = '24';

      const sources = {
        left: ['0', '-30 + 15\\sin(2\\pi t)', '0', '30 + 15\\sin(2\\pi t)', '0', '0', '0'],
        right: ['0', '-30 + 15\\sin(2\\pi t)', '0', '30 + 15\\sin(2\\pi t)', '0', '0', '0'],
      };
      for (const side of SIDES) {
        sources[side].forEach((s, i) => {
          if (this.formulaInputs[side][i]) this.formulaInputs[side][i].value = s;
        });
      }
    } else if (key === 'circle') {
      this.currentSpace = 'CartesianSpace';
      if (spaceSelect) spaceSelect.value = 'CartesianSpace';
      this.rebuildFormulaRows();
      this.tStartInput.value = '0';
      this.tEndInput.value = '1';
      this.durationInput.value = '4.0';
      this.keypointsInput.value = '28';

      const snapshot = this.controller.snapshotValues();
      const lx = snapshot.left.CartesianSpace[0].toFixed(1);
      const rx = snapshot.right.CartesianSpace[0].toFixed(1);

      const sources = {
        left: [`${lx} + 30\\sin(2\\pi t)`, '240 + 30\\cos(2\\pi t) - 30', '0', '30', '0', '-90'],
        right: [`${rx} + 30\\sin(2\\pi t)`, '-240 - 30\\cos(2\\pi t) + 30', '0', '-30', '0', '90'],
      };
      for (const side of SIDES) {
        sources[side].forEach((s, i) => {
          if (this.formulaInputs[side][i]) this.formulaInputs[side][i].value = s;
        });
      }
    } else if (key === 'helix') {
      this.currentSpace = 'CartesianSpace';
      if (spaceSelect) spaceSelect.value = 'CartesianSpace';
      this.rebuildFormulaRows();
      this.tStartInput.value = '0';
      this.tEndInput.value = '1';
      this.durationInput.value = '4.0';
      this.keypointsInput.value = '30';

      const snapshot = this.controller.snapshotValues();
      const lx = snapshot.left.CartesianSpace[0].toFixed(1);
      const rx = snapshot.right.CartesianSpace[0].toFixed(1);

      const sources = {
        left: [`${lx} + 25\\sin(2\\pi t)`, '240 + 25\\cos(2\\pi t) - 25', '40 t', '30', '0', '-90'],
        right: [`${rx} + 25\\sin(2\\pi t)`, '-240 - 25\\cos(2\\pi t) + 25', '40 t', '-30', '0', '90'],
      };
      for (const side of SIDES) {
        sources[side].forEach((s, i) => {
          if (this.formulaInputs[side][i]) this.formulaInputs[side][i].value = s;
        });
      }
    } else if (key === 'wave') {
      this.currentSpace = 'JointAngleSpace';
      if (spaceSelect) spaceSelect.value = 'JointAngleSpace';
      this.rebuildFormulaRows();
      this.tStartInput.value = '0';
      this.tEndInput.value = '1';
      this.durationInput.value = '3.5';
      this.keypointsInput.value = '25';

      const sources = {
        left: ['-20', '-40', '10', '50', '25\\sin(4\\pi t)', '10', '0'],
        right: ['20', '-40', '-10', '50', '-25\\sin(4\\pi t)', '-10', '0'],
      };
      for (const side of SIDES) {
        sources[side].forEach((s, i) => {
          if (this.formulaInputs[side][i]) this.formulaInputs[side][i].value = s;
        });
      }
    }
  }

  private async generate(): Promise<void> {
    const tStart = parseFloat(this.tStartInput.value);
    const tEnd = parseFloat(this.tEndInput.value);
    const durationS = parseFloat(this.durationInput.value);
    const keypointCount = parseInt(this.keypointsInput.value, 10);
    const allowDiscontinuity = this.allowDiscontinuityCheckbox.checked;

    if (!Number.isFinite(tStart) || !Number.isFinite(tEnd) || !Number.isFinite(durationS)) {
      showToast({ title: '参数错误', message: '时间起点、终点与时长必须为有效数值', type: 'danger' });
      return;
    }
    if (tEnd <= tStart) {
      showToast({ title: '参数错误', message: 't 终点必须严格大于 t 起点', type: 'danger' });
      return;
    }
    if (keypointCount < 2 || keypointCount > 500) {
      showToast({ title: '参数错误', message: '采样点数必须在 2 ~ 500 之间', type: 'danger' });
      return;
    }

    const sources: Record<Side, string[]> = {
      left: this.formulaInputs.left.map(inp => inp.value.trim()),
      right: this.formulaInputs.right.map(inp => inp.value.trim()),
    };

    const spinner = this.backdrop?.querySelector('#gen-spinner') as HTMLElement;
    if (spinner) spinner.style.display = 'inline';

    try {
      const generated = await this.controller.addFunctionTrajectory({
        space: this.currentSpace,
        tStart,
        tEnd,
        durationMs: Math.round(durationS * 1000),
        keypointCount,
        sources,
        allowDiscontinuity,
      });

      showToast({
        title: '轨迹生成成功',
        message: `成功生成并追加 ${generated.length} 个动作关键点至时间轴`,
        type: 'success',
      });
      this.close();
      this.onGenerated();
    } catch (err: any) {
      showToast({
        title: '轨迹计算失败',
        message: err.message || '公式求解失败或下游逆运动学不可达',
        type: 'danger',
      });
    } finally {
      if (spinner) spinner.style.display = 'none';
    }
  }

  close(): void {
    if (this.backdrop && this.backdrop.parentElement) {
      this.backdrop.parentElement.removeChild(this.backdrop);
    }
    this.backdrop = null;
  }
}
