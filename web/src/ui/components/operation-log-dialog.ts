import { RAD } from '../../math';
import type { RoboArmController } from '../../controller';
import type { FrameTarget, Side } from '../../types';
import { SIDES } from '../../types';
import { ICONS } from '../icons';

interface OperationLogCallbacks {
  onLocate: (index: number) => void;
  onEdit: (index: number) => void;
}

export class OperationLogDialog {
  private backdrop: HTMLElement | null = null;

  constructor(
    private readonly controller: RoboArmController,
    private readonly callbacks: OperationLogCallbacks,
  ) {}

  open(): void {
    this.close();
    this.backdrop = document.createElement('div');
    this.backdrop.className = 'modal-backdrop';
    const history = this.controller.history;
    this.backdrop.innerHTML = `
      <div class="modal-window" style="max-width: 980px;">
        <div class="modal-header">
          <div class="modal-title">
            <span class="btn-icon-slot">${ICONS.HISTORY}</span>
            <span>历史操作记录</span>
            <span style="font-size: 12px; color: var(--text-muted); font-weight: 500;">${history.length} 个关键点</span>
          </div>
          <button class="modal-close-btn" id="operation-log-close">${ICONS.CLOSE}</button>
        </div>
        <div class="modal-body" style="max-height: 70vh;">
          ${history.length ? history.map((point, index) => `
            <article style="border: 1px solid var(--border-color); border-radius: var(--radius-md); padding: 12px; margin-bottom: 10px; background: var(--bg-card);">
              <div style="display: flex; align-items: center; justify-content: space-between; gap: 12px;">
                <div>
                  <strong>#${index + 1} ${escapeHtml(point.label)}</strong>
                  <span style="margin-left: 8px; color: var(--text-muted); font-size: 12px;">${point.durationMs} ms · timeout ${point.timeoutMs} ms</span>
                </div>
                <div style="display: flex; gap: 6px;">
                  <button class="btn btn-ghost btn-sm" data-locate="${index}">定位</button>
                  <button class="btn btn-secondary btn-sm" data-edit="${index}">${ICONS.EDIT} 编辑</button>
                </div>
              </div>
              <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 9px; font-family: var(--font-mono); font-size: 11px; color: var(--text-secondary);">
                ${SIDES.map(side => `<div><b>${side === 'left' ? '左臂' : '右臂'}</b> · ${this.targetText(side, point.frameTargets[side])}</div>`).join('')}
              </div>
              ${point.function ? `
                <details style="margin-top: 9px; font-size: 12px; color: var(--text-secondary);">
                  <summary style="cursor: pointer;">函数轨迹 ${point.function.group} · t=${point.function.t.toPrecision(6)} · ${point.function.space}</summary>
                  <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 7px; font-family: var(--font-mono); font-size: 11px; overflow-wrap: anywhere;">
                    <div><b>左臂公式</b><br>${point.function.sources.left.map(escapeHtml).join('<br>')}</div>
                    <div><b>右臂公式</b><br>${point.function.sources.right.map(escapeHtml).join('<br>')}</div>
                  </div>
                </details>` : ''}
            </article>`).join('') : '<div class="alert-banner info">当前还没有历史操作记录。</div>'}
        </div>
        <div class="modal-footer"><button class="btn btn-secondary" id="operation-log-done">关闭</button></div>
      </div>`;
    document.body.appendChild(this.backdrop);
    this.backdrop.querySelector('#operation-log-close')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#operation-log-done')?.addEventListener('click', () => this.close());
    this.backdrop.querySelectorAll<HTMLElement>('[data-locate]').forEach(button => button.addEventListener('click', () => {
      const index = Number(button.dataset.locate); this.close(); this.callbacks.onLocate(index);
    }));
    this.backdrop.querySelectorAll<HTMLElement>('[data-edit]').forEach(button => button.addEventListener('click', () => {
      const index = Number(button.dataset.edit); this.close(); this.callbacks.onEdit(index);
    }));
  }

  private targetText(_side: Side, frame: FrameTarget): string {
    if (frame.space === 'JointAngleSpace') {
      return `Joint [${frame.target.jointAngles.map(value => (value * RAD).toFixed(1)).join(', ')}]°`;
    }
    const target = frame.target;
    return `Cartesian [${target.x.toFixed(1)}, ${target.y.toFixed(1)}, ${target.z.toFixed(1)}] mm · YPR [${(target.yaw * RAD).toFixed(1)}, ${(target.pitch * RAD).toFixed(1)}, ${(target.roll * RAD).toFixed(1)}]°`;
  }

  close(): void { this.backdrop?.remove(); this.backdrop = null; }
}

function escapeHtml(value: string): string {
  return value.replace(/[&<>"']/g, character => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  }[character] ?? character));
}
