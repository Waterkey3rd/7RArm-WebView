import type { ArmRenderer, CameraPreset } from '../../renderer';
import type { ArmState, Side } from '../../types';
import type { DeployIK } from '../../wasm';
import { matrixToYpr, RAD } from '../../math';
import { ICONS } from '../icons';

export interface ViewportHudCallbacks {
  onSelectCamera: (preset: CameraPreset) => void;
  onResetCamera: () => void;
  onToggleGrid: () => boolean;
  onToggleAxes: () => boolean;
  onToggleTool: () => boolean;
}

export class ViewportHud {
  readonly element: HTMLElement;
  private leftReadout: HTMLElement;
  private rightReadout: HTMLElement;
  private motionBanner: HTMLElement;
  private motionLabel: HTMLElement;

  constructor(
    private readonly container: HTMLElement,
    private readonly ik: DeployIK,
    private readonly callbacks: ViewportHudCallbacks,
  ) {
    this.element = document.createElement('div');
    this.element.className = 'viewport-hud-root';

    this.element.innerHTML = `
      <!-- Top Left: Floating Pose HUD -->
      <div class="viewport-overlay-top-left">
        <div class="hud-panel">
          <div class="hud-arm-readout" id="hud-left-arm">
            <span class="hud-chip left">左臂</span>
            <span class="hud-coords" id="hud-left-coords">X: 0.0 Y: 0.0 Z: 0.0 mm | Y: 0° P: 0° R: 0°</span>
          </div>
          <div class="hud-arm-readout" id="hud-right-arm">
            <span class="hud-chip right">右臂</span>
            <span class="hud-coords" id="hud-right-coords">X: 0.0 Y: 0.0 Z: 0.0 mm | Y: 0° P: 0° R: 0°</span>
          </div>
        </div>
      </div>

      <!-- Center Top: Motion Progress Overlay -->
      <div class="motion-progress-banner" id="motion-progress-banner">
        <span class="motion-spinner">${ICONS.SPINNER}</span>
        <span id="motion-progress-text">运动执行中... 0%</span>
      </div>

      <!-- Bottom Left: Camera & Layer Toolbar -->
      <div class="viewport-overlay-bottom-left">
        <button class="btn btn-ghost btn-sm" data-cam="iso" title="等轴透视视角">透视</button>
        <button class="btn btn-ghost btn-sm" data-cam="top" title="正上方顶视图">顶视</button>
        <button class="btn btn-ghost btn-sm" data-cam="front" title="正前方视图">正视</button>
        <button class="btn btn-ghost btn-sm" data-cam="side" title="右侧侧视图">侧视</button>
        <button class="btn btn-ghost btn-sm" id="btn-cam-reset" title="复位视角">
          <span style="margin-right: 2px;">${ICONS.RESET}</span>
          <span>复位</span>
        </button>
        <span style="width: 1px; height: 16px; background: var(--border-color); margin: 0 4px;"></span>
        <button class="btn btn-ghost btn-sm" id="btn-layer-grid" title="切换地面参考网格">
          <span style="margin-right: 2px;">${ICONS.GRID}</span>
          <span>网格</span>
        </button>
        <button class="btn btn-ghost btn-sm" id="btn-layer-axes" title="切换关节转轴指示器">
          <span style="margin-right: 2px;">${ICONS.AXES}</span>
          <span>关节轴</span>
        </button>
        <button class="btn btn-ghost btn-sm" id="btn-layer-tool" title="切换末端工具坐标系">
          <span style="margin-right: 2px;">${ICONS.TOOL}</span>
          <span>末端系</span>
        </button>
      </div>
    `;

    container.appendChild(this.element);

    this.leftReadout = this.element.querySelector('#hud-left-coords') as HTMLElement;
    this.rightReadout = this.element.querySelector('#hud-right-coords') as HTMLElement;
    this.motionBanner = this.element.querySelector('#motion-progress-banner') as HTMLElement;
    this.motionLabel = this.element.querySelector('#motion-progress-text') as HTMLElement;

    // Bind Camera presets
    this.element.querySelectorAll<HTMLButtonElement>('[data-cam]').forEach(btn => {
      btn.addEventListener('click', () => {
        const cam = btn.dataset.cam as CameraPreset;
        this.callbacks.onSelectCamera(cam);
      });
    });

    this.element.querySelector('#btn-cam-reset')?.addEventListener('click', () => {
      this.callbacks.onResetCamera();
    });

    // Bind Layer toggles
    this.element.querySelector('#btn-layer-grid')?.addEventListener('click', (e) => {
      const active = this.callbacks.onToggleGrid();
      (e.currentTarget as HTMLElement).style.color = active ? 'var(--left-arm-primary)' : 'var(--text-muted)';
    });

    this.element.querySelector('#btn-layer-axes')?.addEventListener('click', (e) => {
      const active = this.callbacks.onToggleAxes();
      (e.currentTarget as HTMLElement).style.color = active ? 'var(--left-arm-primary)' : 'var(--text-muted)';
    });

    this.element.querySelector('#btn-layer-tool')?.addEventListener('click', (e) => {
      const active = this.callbacks.onToggleTool();
      (e.currentTarget as HTMLElement).style.color = active ? 'var(--left-arm-primary)' : 'var(--text-muted)';
    });
  }

  update(state: ArmState): void {
    for (const side of ['left', 'right'] as Side[]) {
      const fk = this.ik.fk(side, state[side]);
      const [yaw, pitch, roll] = matrixToYpr(fk.rotation);
      const str = `X: ${fk.position[0].toFixed(1)} Y: ${fk.position[1].toFixed(1)} Z: ${fk.position[2].toFixed(1)} mm | Y: ${(yaw * RAD).toFixed(1)}° P: ${(pitch * RAD).toFixed(1)}° R: ${(roll * RAD).toFixed(1)}°`;
      if (side === 'left') {
        this.leftReadout.textContent = str;
      } else {
        this.rightReadout.textContent = str;
      }
    }
  }

  showMotionProgress(visible: boolean, text = ''): void {
    this.motionBanner.style.display = visible ? 'flex' : 'none';
    if (text) this.motionLabel.textContent = text;
  }
}
