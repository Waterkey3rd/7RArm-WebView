import type { RoboArmController } from '../../controller';
import { DEG, RAD, matrixToYpr, rpyMatrix } from '../../math';
import type { ArmState, Side } from '../../types';
import { SIDES } from '../../types';
import type { DeployIK } from '../../wasm';
import { showToast } from './toast';

export type ControlMode = 'joint' | 'cartesian' | 'delta';

export interface ControlPanelCallbacks {
  onExecuteJoint: (valuesDeg: Record<Side, number[]>, durationMs: number) => void;
  onExecuteCartesian: (values: Record<Side, number[]>, durationMs: number) => void;
  onExecuteDelta: (values: Record<Side, number[]>, durationMs: number) => void;
  onUndo: () => void;
  onReset: () => void;
  onPreviewStateChange?: (state: ArmState) => void;
}

export class ControlPanel {
  readonly element: HTMLElement;
  private currentMode: ControlMode = 'joint';

  // Joint state inputs
  private jointInputs: Record<Side, HTMLInputElement[]> = { left: [], right: [] };
  private jointSliders: Record<Side, HTMLInputElement[]> = { left: [], right: [] };
  private jointGauges: Record<Side, HTMLElement[]> = { left: [], right: [] };

  // Cartesian inputs
  private cartInputs: Record<Side, HTMLInputElement[]> = { left: [], right: [] };
  private cartIkBadges: Record<Side, HTMLElement> = {} as any;

  // Delta inputs
  private deltaStepMm = 10;
  private deltaStepDeg = 10;

  // Limits in deg
  private limitsDeg: Record<Side, { lower: number[]; upper: number[] }> = {} as any;

  private durationSlider!: HTMLInputElement;
  private durationLabel!: HTMLElement;
  private executeBtn!: HTMLButtonElement;

  constructor(
    private readonly container: HTMLElement,
    private readonly controller: RoboArmController,
    private readonly callbacks: ControlPanelCallbacks,
  ) {
    for (const side of SIDES) {
      const limits = this.controller.ik.limits(side);
      this.limitsDeg[side] = {
        lower: limits.lower.map(v => v * RAD),
        upper: limits.upper.map(v => v * RAD),
      };
    }

    this.element = document.createElement('div');
    this.element.className = 'control-dock';

    this.element.innerHTML = `
      <!-- Dock Header: Segmented Nav -->
      <div class="dock-header">
        <div class="segmented-nav">
          <button class="segment-btn active" data-mode="joint">
            <span>⚙️</span>
            <span>关节角</span>
          </button>
          <button class="segment-btn" data-mode="cartesian">
            <span>📐</span>
            <span>笛卡尔</span>
          </button>
          <button class="segment-btn" data-mode="delta">
            <span>🎯</span>
            <span>Delta 微调</span>
          </button>
        </div>
      </div>

      <!-- Scrollable Dock Content -->
      <div class="dock-scroll-area" id="dock-content-area">
        <!-- Will be populated dynamically based on mode -->
      </div>

      <!-- Dock Footer: Execution Bar -->
      <div class="dock-footer">
        <div style="display: flex; align-items: center; justify-content: space-between; font-size: 12px; font-weight: 600;">
          <span style="color: var(--text-muted);">运动过渡时长</span>
          <span style="font-family: var(--font-mono); color: var(--left-arm-primary);" id="duration-val-display">2.0 秒</span>
        </div>
        <input type="range" class="joint-slider" id="motion-duration-slider" min="0.3" max="8.0" step="0.1" value="2.0">

        <div style="display: flex; gap: 8px; margin-top: 4px;">
          <button class="btn btn-primary btn-lg" id="btn-execute-motion" style="flex: 2;" title="执行当前姿态运动 (Ctrl+Enter)">
            <span>▶ 执行运动</span>
          </button>
          <button class="btn btn-secondary btn-lg" id="btn-undo-motion" style="flex: 1;" title="撤销回退至上一状态 (Ctrl+Z)">
            <span>↶ 回退</span>
          </button>
          <button class="btn btn-secondary btn-icon btn-lg" id="btn-reset-motion" title="重置机械臂到零位">
            <span>🔄</span>
          </button>
        </div>
      </div>
    `;

    container.appendChild(this.element);

    this.durationSlider = this.element.querySelector('#motion-duration-slider') as HTMLInputElement;
    this.durationLabel = this.element.querySelector('#duration-val-display') as HTMLElement;
    this.executeBtn = this.element.querySelector('#btn-execute-motion') as HTMLButtonElement;

    this.durationSlider.addEventListener('input', () => {
      this.durationLabel.textContent = `${parseFloat(this.durationSlider.value).toFixed(1)} 秒`;
    });

    // Segmented Nav listeners
    this.element.querySelectorAll<HTMLButtonElement>('.segment-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        this.element.querySelectorAll('.segment-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        this.setMode(btn.dataset.mode as ControlMode);
      });
    });

    // Execute & Undo
    this.executeBtn.addEventListener('click', () => this.handleExecute());
    this.element.querySelector('#btn-undo-motion')?.addEventListener('click', () => this.callbacks.onUndo());
    this.element.querySelector('#btn-reset-motion')?.addEventListener('click', () => this.callbacks.onReset());

    // Initial render
    this.renderModeContent();
  }

  private setMode(mode: ControlMode): void {
    this.currentMode = mode;
    this.renderModeContent();
  }

  private getDurationMs(): number {
    return Math.round(parseFloat(this.durationSlider.value) * 1000);
  }

  private renderModeContent(): void {
    const area = this.element.querySelector('#dock-content-area') as HTMLElement;
    area.innerHTML = '';

    if (this.currentMode === 'joint') {
      this.renderJointMode(area);
    } else if (this.currentMode === 'cartesian') {
      this.renderCartesianMode(area);
    } else if (this.currentMode === 'delta') {
      this.renderDeltaMode(area);
    }
  }

  // ==========================================
  // Mode 1: Joint Space
  // ==========================================
  private renderJointMode(parent: HTMLElement): void {
    const currentState = this.controller.current;
    this.jointInputs = { left: [], right: [] };
    this.jointSliders = { left: [], right: [] };
    this.jointGauges = { left: [], right: [] };

    // Pose Presets Bar
    const presetBar = document.createElement('div');
    presetBar.style.display = 'flex';
    presetBar.style.gap = '6px';
    presetBar.innerHTML = `
      <button class="btn btn-secondary btn-sm" style="flex: 1;" id="btn-preset-zero">零位姿态</button>
      <button class="btn btn-secondary btn-sm" style="flex: 1;" id="btn-preset-ready">准备姿态</button>
      <button class="btn btn-secondary btn-sm" style="flex: 1;" id="btn-preset-mirror" title="将左臂关节镜像复制到右臂">左向右镜像</button>
    `;
    parent.appendChild(presetBar);

    presetBar.querySelector('#btn-preset-zero')?.addEventListener('click', () => {
      for (const side of SIDES) {
        for (let i = 0; i < 7; i++) {
          this.setJointValue(side, i, 0);
        }
      }
    });

    presetBar.querySelector('#btn-preset-ready')?.addEventListener('click', () => {
      const readyPose = [0, -30, 0, 30, 0, 0, 0];
      for (const side of SIDES) {
        for (let i = 0; i < 7; i++) {
          this.setJointValue(side, i, readyPose[i]);
        }
      }
    });

    presetBar.querySelector('#btn-preset-mirror')?.addEventListener('click', () => {
      for (let i = 0; i < 7; i++) {
        const val = parseFloat(this.jointInputs.left[i].value) || 0;
        this.setJointValue('right', i, val);
      }
      showToast({ message: '已完成左臂关节角镜像复制', type: 'info' });
    });

    for (const side of SIDES) {
      const card = document.createElement('div');
      card.className = `arm-card ${side}-arm`;

      const sideName = side === 'left' ? '左臂 (Left Arm)' : '右臂 (Right Arm)';
      card.innerHTML = `
        <div class="arm-card-header">
          <span>${sideName}</span>
          <span style="font-size: 11px; font-weight: normal; opacity: 0.8;">J1 ~ J7 (deg)</span>
        </div>
        <div class="arm-card-content" id="joints-host-${side}"></div>
      `;
      parent.appendChild(card);

      const host = card.querySelector(`#joints-host-${side}`) as HTMLElement;

      for (let i = 0; i < 7; i++) {
        const row = document.createElement('div');
        row.className = 'joint-row';

        const min = this.limitsDeg[side].lower[i];
        const max = this.limitsDeg[side].upper[i];
        const currentDeg = currentState[side][i] * RAD;

        row.innerHTML = `
          <div class="joint-header">
            <span class="joint-name">J${i + 1}</span>
            <div style="display: flex; align-items: center; gap: 6px;">
              <span style="font-size: 10px; color: var(--text-muted); font-family: var(--font-mono);">${min.toFixed(0)}° ~ ${max.toFixed(0)}°</span>
              <input type="number" class="joint-val-badge" id="num-${side}-${i}" step="0.5" min="${min.toFixed(1)}" max="${max.toFixed(1)}" value="${currentDeg.toFixed(1)}">
            </div>
          </div>
          <div class="joint-control-line">
            <div class="stepper-group">
              <button class="stepper-btn" data-step="-5">-5°</button>
              <button class="stepper-btn" data-step="-1">-1°</button>
            </div>
            <input type="range" class="joint-slider" id="slider-${side}-${i}" min="${min}" max="${max}" step="0.1" value="${currentDeg}">
            <div class="stepper-group">
              <button class="stepper-btn" data-step="1">+1°</button>
              <button class="stepper-btn" data-step="5">+5°</button>
            </div>
          </div>
          <div class="limit-bar-track">
            <div class="limit-bar-fill limit-safe" id="gauge-${side}-${i}" style="width: 50%;"></div>
          </div>
        `;

        host.appendChild(row);

        const slider = row.querySelector(`#slider-${side}-${i}`) as HTMLInputElement;
        const num = row.querySelector(`#num-${side}-${i}`) as HTMLInputElement;
        const gauge = row.querySelector(`#gauge-${side}-${i}`) as HTMLElement;

        this.jointSliders[side].push(slider);
        this.jointInputs[side].push(num);
        this.jointGauges[side].push(gauge);

        this.updateJointGauge(side, i, currentDeg);

        slider.addEventListener('input', () => {
          const val = parseFloat(slider.value);
          num.value = val.toFixed(1);
          this.updateJointGauge(side, i, val);
        });

        num.addEventListener('change', () => {
          let val = parseFloat(num.value);
          if (isNaN(val)) val = 0;
          val = Math.max(min, Math.min(max, val));
          num.value = val.toFixed(1);
          slider.value = String(val);
          this.updateJointGauge(side, i, val);
        });

        row.querySelectorAll<HTMLButtonElement>('.stepper-btn').forEach(btn => {
          btn.addEventListener('click', () => {
            const step = parseFloat(btn.dataset.step || '0');
            const cur = parseFloat(slider.value) || 0;
            const target = Math.max(min, Math.min(max, cur + step));
            this.setJointValue(side, i, target);
          });
        });
      }
    }
  }

  private setJointValue(side: Side, index: number, deg: number): void {
    const min = this.limitsDeg[side].lower[index];
    const max = this.limitsDeg[side].upper[index];
    const clamped = Math.max(min, Math.min(max, deg));
    if (this.jointSliders[side][index]) {
      this.jointSliders[side][index].value = String(clamped);
      this.jointInputs[side][index].value = clamped.toFixed(1);
      this.updateJointGauge(side, index, clamped);
    }
  }

  private updateJointGauge(side: Side, index: number, deg: number): void {
    const min = this.limitsDeg[side].lower[index];
    const max = this.limitsDeg[side].upper[index];
    const ratio = Math.max(0, Math.min(1, (deg - min) / (max - min)));
    const gauge = this.jointGauges[side][index];
    if (!gauge) return;

    gauge.style.width = `${(ratio * 100).toFixed(1)}%`;
    if (ratio < 0.08 || ratio > 0.92) {
      gauge.className = 'limit-bar-fill limit-danger';
    } else if (ratio < 0.18 || ratio > 0.82) {
      gauge.className = 'limit-bar-fill limit-warn';
    } else {
      gauge.className = 'limit-bar-fill limit-safe';
    }
  }

  // ==========================================
  // Mode 2: Cartesian Space
  // ==========================================
  private renderCartesianMode(parent: HTMLElement): void {
    this.cartInputs = { left: [], right: [] };
    const snapshot = this.controller.snapshotValues();

    for (const side of SIDES) {
      const card = document.createElement('div');
      card.className = `arm-card ${side}-arm`;
      const sideName = side === 'left' ? '左臂末端笛卡尔位姿' : '右臂末端笛卡尔位姿';

      card.innerHTML = `
        <div class="arm-card-header">
          <span>${sideName}</span>
          <span style="font-size: 11px; font-weight: normal; opacity: 0.8;">X,Y,Z (mm) / Y,P,R (deg)</span>
        </div>
        <div class="arm-card-content">
          <div class="coord-grid">
            <div class="coord-item">
              <span class="coord-label">X (mm)</span>
              <input type="number" class="coord-input" id="cart-${side}-0" step="5">
            </div>
            <div class="coord-item">
              <span class="coord-label">Y (mm)</span>
              <input type="number" class="coord-input" id="cart-${side}-1" step="5">
            </div>
            <div class="coord-item">
              <span class="coord-label">Z (mm)</span>
              <input type="number" class="coord-input" id="cart-${side}-2" step="5">
            </div>
            <div class="coord-item">
              <span class="coord-label">Yaw (deg)</span>
              <input type="number" class="coord-input" id="cart-${side}-3" step="5">
            </div>
            <div class="coord-item">
              <span class="coord-label">Pitch (deg)</span>
              <input type="number" class="coord-input" id="cart-${side}-4" step="5">
            </div>
            <div class="coord-item">
              <span class="coord-label">Roll (deg)</span>
              <input type="number" class="coord-input" id="cart-${side}-5" step="5">
            </div>
          </div>
          <div class="ik-probe-box reachable" id="cart-probe-${side}">
            <span id="cart-probe-icon-${side}">✅</span>
            <span id="cart-probe-text-${side}">IK 可达解算正常</span>
          </div>
        </div>
      `;

      parent.appendChild(card);

      const cartVals = snapshot[side].CartesianSpace;
      for (let i = 0; i < 6; i++) {
        const inp = card.querySelector(`#cart-${side}-${i}`) as HTMLInputElement;
        inp.value = cartVals[i].toFixed(1);
        this.cartInputs[side].push(inp);
        inp.addEventListener('input', () => this.probeCartesianIK(side));
      }

      this.cartIkBadges[side] = card.querySelector(`#cart-probe-${side}`) as HTMLElement;
      this.probeCartesianIK(side);
    }
  }

  private probeCartesianIK(side: Side): boolean {
    const vals = this.cartInputs[side].map(inp => parseFloat(inp.value) || 0);
    const badge = this.cartIkBadges[side];
    if (!badge) return true;

    try {
      const rot = rpyMatrix(vals[5] * DEG, vals[4] * DEG, vals[3] * DEG);
      const pos = vals.slice(0, 3);
      const cur = this.controller.current[side];
      const sol = this.controller.ik.solve(side, rot, pos, cur);

      badge.className = 'ik-probe-box reachable';
      badge.querySelector(`#cart-probe-icon-${side}`)!.textContent = '✅';
      badge.querySelector(`#cart-probe-text-${side}`)!.textContent =
        `IK 解算可达 · 误差 ${sol.positionErrorMm.toFixed(3)} mm / ${(sol.orientationErrorRad * RAD).toFixed(2)}°`;
      return true;
    } catch (e: any) {
      badge.className = 'ik-probe-box unreachable';
      badge.querySelector(`#cart-probe-icon-${side}`)!.textContent = '❌';
      badge.querySelector(`#cart-probe-text-${side}`)!.textContent = e.message || '超出工作空间/无有效逆解';
      return false;
    }
  }

  // ==========================================
  // Mode 3: Delta Jogging
  // ==========================================
  private renderDeltaMode(parent: HTMLElement): void {
    // Step selector
    const stepBox = document.createElement('div');
    stepBox.className = 'arm-card';
    stepBox.style.padding = '10px 14px';
    stepBox.innerHTML = `
      <div style="font-size: 12px; font-weight: 700; margin-bottom: 6px; color: var(--text-secondary);">微调步长选择 (Step Size)</div>
      <div style="display: flex; gap: 6px;">
        <button class="btn btn-secondary btn-sm" data-step="1" style="flex: 1;">1 mm / 1°</button>
        <button class="btn btn-secondary btn-sm" data-step="5" style="flex: 1;">5 mm / 5°</button>
        <button class="btn btn-primary btn-sm" data-step="10" style="flex: 1;">10 mm / 10°</button>
        <button class="btn btn-secondary btn-sm" data-step="25" style="flex: 1;">25 mm / 25°</button>
      </div>
    `;
    parent.appendChild(stepBox);

    stepBox.querySelectorAll<HTMLButtonElement>('[data-step]').forEach(btn => {
      btn.addEventListener('click', () => {
        stepBox.querySelectorAll('.btn').forEach(b => {
          b.classList.remove('btn-primary');
          b.classList.add('btn-secondary');
        });
        btn.classList.remove('btn-secondary');
        btn.classList.add('btn-primary');
        const s = parseFloat(btn.dataset.step || '10');
        this.deltaStepMm = s;
        this.deltaStepDeg = s;
      });
    });

    for (const side of SIDES) {
      const card = document.createElement('div');
      card.className = `arm-card ${side}-arm`;
      const sideName = side === 'left' ? '左臂末端工具系微调' : '右臂末端工具系微调';

      card.innerHTML = `
        <div class="arm-card-header">
          <span>${sideName}</span>
          <span style="font-size: 11px; font-weight: normal; opacity: 0.8;">Tool-Frame Relative</span>
        </div>
        <div class="arm-card-content">
          <div style="font-size: 11px; font-weight: 700; color: var(--text-muted);">位置移动 (ΔX, ΔY, ΔZ)</div>
          <div class="jog-pad-grid">
            <button class="jog-key" data-side="${side}" data-axis="0" data-dir="1">+X 前</button>
            <button class="jog-key" data-side="${side}" data-axis="1" data-dir="1">+Y 左</button>
            <button class="jog-key" data-side="${side}" data-axis="2" data-dir="1">+Z 上</button>
            <button class="jog-key" data-side="${side}" data-axis="0" data-dir="-1">-X 后</button>
            <button class="jog-key" data-side="${side}" data-axis="1" data-dir="-1">-Y 右</button>
            <button class="jog-key" data-side="${side}" data-axis="2" data-dir="-1">-Z 下</button>
          </div>

          <div style="font-size: 11px; font-weight: 700; color: var(--text-muted); margin-top: 6px;">姿态旋转 (ΔYaw, ΔPitch, ΔRoll)</div>
          <div class="jog-pad-grid">
            <button class="jog-key" data-side="${side}" data-axis="3" data-dir="1">+Yaw 偏航</button>
            <button class="jog-key" data-side="${side}" data-axis="4" data-dir="1">+Pitch 俯仰</button>
            <button class="jog-key" data-side="${side}" data-axis="5" data-dir="1">+Roll 滚转</button>
            <button class="jog-key" data-side="${side}" data-axis="3" data-dir="-1">-Yaw 偏航</button>
            <button class="jog-key" data-side="${side}" data-axis="4" data-dir="-1">-Pitch 俯仰</button>
            <button class="jog-key" data-side="${side}" data-axis="5" data-dir="-1">-Roll 滚转</button>
          </div>
        </div>
      `;

      parent.appendChild(card);

      card.querySelectorAll<HTMLButtonElement>('.jog-key').forEach(btn => {
        btn.addEventListener('click', () => {
          const s = btn.dataset.side as Side;
          const axis = parseInt(btn.dataset.axis || '0', 10);
          const dir = parseInt(btn.dataset.dir || '1', 10);
          this.handleDeltaJog(s, axis, dir);
        });
      });
    }
  }

  private handleDeltaJog(side: Side, axis: number, dir: number): void {
    const delta = { left: [0, 0, 0, 0, 0, 0], right: [0, 0, 0, 0, 0, 0] };
    const step = axis < 3 ? this.deltaStepMm : this.deltaStepDeg;
    delta[side][axis] = step * dir;
    this.callbacks.onExecuteDelta(delta, this.getDurationMs());
  }

  // ==========================================
  // Execution trigger
  // ==========================================
  handleExecute(): void {
    const duration = this.getDurationMs();

    if (this.currentMode === 'joint') {
      const values: Record<Side, number[]> = {
        left: this.jointInputs.left.map(i => parseFloat(i.value) || 0),
        right: this.jointInputs.right.map(i => parseFloat(i.value) || 0),
      };
      this.callbacks.onExecuteJoint(values, duration);
    } else if (this.currentMode === 'cartesian') {
      const values: Record<Side, number[]> = {
        left: this.cartInputs.left.map(i => parseFloat(i.value) || 0),
        right: this.cartInputs.right.map(i => parseFloat(i.value) || 0),
      };
      this.callbacks.onExecuteCartesian(values, duration);
    }
  }

  // Sync controls with robot current state
  syncState(state: ArmState): void {
    if (this.currentMode === 'joint') {
      for (const side of SIDES) {
        for (let i = 0; i < 7; i++) {
          const deg = state[side][i] * RAD;
          if (this.jointSliders[side][i]) {
            this.jointSliders[side][i].value = String(deg);
            this.jointInputs[side][i].value = deg.toFixed(1);
            this.updateJointGauge(side, i, deg);
          }
        }
      }
    } else if (this.currentMode === 'cartesian') {
      const snapshot = this.controller.snapshotValues(state);
      for (const side of SIDES) {
        const c = snapshot[side].CartesianSpace;
        for (let i = 0; i < 6; i++) {
          if (this.cartInputs[side][i]) {
            this.cartInputs[side][i].value = c[i].toFixed(1);
          }
        }
        this.probeCartesianIK(side);
      }
    }
  }
}
