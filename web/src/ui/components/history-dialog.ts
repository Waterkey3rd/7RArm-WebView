import type { RoboArmController } from '../../controller';
import { DEG, RAD, matrixToYpr } from '../../math';
import { cartesianTarget, jointTarget } from '../../sequence';
import { SIDES, type FrameTarget, type HistoryPoint, type Side, type Space } from '../../types';
import { showToast } from './toast';

export class HistoryDialog {
  private backdrop: HTMLElement | null = null;
  private currentSpaces: Record<Side, Space> = { left: 'JointAngleSpace', right: 'JointAngleSpace' };
  private valueInputs: Record<Side, HTMLInputElement[]> = { left: [], right: [] };

  constructor(
    private readonly controller: RoboArmController,
    private readonly onSaved: () => void,
  ) {}

  open(index: number): void {
    const point = this.controller.history[index];
    if (!point) return;

    if (this.backdrop) this.close();

    this.backdrop = document.createElement('div');
    this.backdrop.className = 'modal-backdrop';

    this.currentSpaces = {
      left: point.frameTargets.left.space,
      right: point.frameTargets.right.space,
    };

    this.backdrop.innerHTML = `
      <div class="modal-window" style="max-width: 780px;">
        <div class="modal-header">
          <div class="modal-title">
            <span>✏️</span>
            <span>编辑动作关键点 #${index + 1}</span>
          </div>
          <button class="modal-close-btn" id="modal-close-btn">×</button>
        </div>

        <div class="modal-body">
          <div style="display: grid; grid-template-columns: 2fr 1fr 1fr; gap: 12px;">
            <div class="form-field">
              <label class="form-field-label">关键点名称</label>
              <input type="text" class="form-input" id="edit-kp-label" value="${escapeHtml(point.label)}">
            </div>
            <div class="form-field">
              <label class="form-field-label">过渡时长 (ms)</label>
              <input type="number" class="form-input" id="edit-kp-duration" value="${point.durationMs}" step="50" min="1">
            </div>
            <div class="form-field">
              <label class="form-field-label">超时时间 (ms)</label>
              <input type="number" class="form-input" id="edit-kp-timeout" value="${point.timeoutMs || 0}" step="50" min="0">
            </div>
          </div>

          <div class="alert-banner info">
            <span>💡 修改该关键点后，系统将自动以后续新姿态重新级联解算（recomputeHistory）所有下游关键点。若发生奇异或超限将自动安全回滚。</span>
          </div>

          <div class="formula-arms-container" id="edit-arms-container"></div>
        </div>

        <div class="modal-footer">
          <button class="btn btn-secondary" id="btn-edit-cancel">取消</button>
          <button class="btn btn-primary" id="btn-edit-save">保存修改并重算</button>
        </div>
      </div>
    `;

    document.body.appendChild(this.backdrop);

    this.backdrop.querySelector('#modal-close-btn')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#btn-edit-cancel')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#btn-edit-save')?.addEventListener('click', () => this.save(index));

    this.renderArmEditors(point);
  }

  private renderArmEditors(point: HistoryPoint): void {
    const host = this.backdrop?.querySelector('#edit-arms-container');
    if (!host) return;
    host.innerHTML = '';
    this.valueInputs = { left: [], right: [] };

    for (const side of SIDES) {
      const box = document.createElement('div');
      box.className = 'formula-arm-box';
      const title = side === 'left' ? '左臂目标设定' : '右臂目标设定';

      box.innerHTML = `
        <div style="display: flex; align-items: center; justify-content: space-between; margin-bottom: 6px;">
          <span class="formula-arm-title ${side}">${title}</span>
          <select class="form-select" id="sel-space-${side}" style="padding: 2px 6px; font-size: 11px;">
            <option value="JointAngleSpace" ${this.currentSpaces[side] === 'JointAngleSpace' ? 'selected' : ''}>关节空间 (deg)</option>
            <option value="CartesianSpace" ${this.currentSpaces[side] === 'CartesianSpace' ? 'selected' : ''}>笛卡尔空间 (mm/deg)</option>
          </select>
        </div>
        <div id="arm-vals-host-${side}" style="display: flex; flex-direction: column; gap: 6px;"></div>
      `;

      host.appendChild(box);

      const spaceSelect = box.querySelector(`#sel-space-${side}`) as HTMLSelectElement;
      spaceSelect.addEventListener('change', () => {
        this.currentSpaces[side] = spaceSelect.value as Space;
        this.populateArmFields(side, point, box.querySelector(`#arm-vals-host-${side}`) as HTMLElement);
      });

      this.populateArmFields(side, point, box.querySelector(`#arm-vals-host-${side}`) as HTMLElement);
    }
  }

  private populateArmFields(side: Side, point: HistoryPoint, container: HTMLElement): void {
    container.innerHTML = '';
    this.valueInputs[side] = [];

    const space = this.currentSpaces[side];
    const isJoint = space === 'JointAngleSpace';
    const labels = isJoint
      ? ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']
      : ['X', 'Y', 'Z', 'Yaw', 'Pitch', 'Roll'];

    let initialValues: number[] = [];
    if (point.frameTargets[side].space === space) {
      if (isJoint) {
        initialValues = (point.frameTargets[side].target as any).jointAngles.map((q: number) => q * RAD);
      } else {
        const t = (point.frameTargets[side].target as any);
        initialValues = [t.x, t.y, t.z, t.yaw * RAD, t.pitch * RAD, t.roll * RAD];
      }
    } else {
      // Convert from current point.target
      if (isJoint) {
        initialValues = point.target[side].map(q => q * RAD);
      } else {
        const fk = this.controller.ik.fk(side, point.target[side]);
        const [yaw, pitch, roll] = matrixToYpr(fk.rotation);
        initialValues = [...fk.position, yaw * RAD, pitch * RAD, roll * RAD];
      }
    }

    labels.forEach((lab, i) => {
      const row = document.createElement('div');
      row.className = 'formula-row';
      row.innerHTML = `
        <span class="formula-var-label">${lab} =</span>
        <input type="number" class="formula-input" id="val-${side}-${i}" value="${(initialValues[i] || 0).toFixed(2)}" step="0.5">
      `;
      container.appendChild(row);

      const inp = row.querySelector(`#val-${side}-${i}`) as HTMLInputElement;
      this.valueInputs[side].push(inp);
    });
  }

  private save(index: number): void {
    const labelInp = this.backdrop?.querySelector('#edit-kp-label') as HTMLInputElement;
    const durInp = this.backdrop?.querySelector('#edit-kp-duration') as HTMLInputElement;
    const timeoutInp = this.backdrop?.querySelector('#edit-kp-timeout') as HTMLInputElement;

    const label = labelInp.value.trim() || `关键点 #${index + 1}`;
    const durationMs = parseInt(durInp.value, 10);
    const timeoutMs = parseInt(timeoutInp.value, 10) || 0;

    if (!Number.isFinite(durationMs) || durationMs < 1) {
      showToast({ title: '参数错误', message: '过渡时长必须不小于 1 ms', type: 'danger' });
      return;
    }

    const targets: Partial<Record<Side, FrameTarget>> = {};

    for (const side of SIDES) {
      const vals = this.valueInputs[side].map(inp => parseFloat(inp.value) || 0);
      if (this.currentSpaces[side] === 'JointAngleSpace') {
        targets[side] = jointTarget(vals.map(v => v * DEG));
      } else {
        const cart = [...vals];
        for (let i = 3; i < 6; i++) cart[i] *= DEG;
        targets[side] = cartesianTarget(cart);
      }
    }

    try {
      this.controller.editHistory(index, {
        label,
        durationMs,
        timeoutMs,
        targets,
      });

      showToast({
        title: '修改成功',
        message: `已更新关键点 #${index + 1} 并完成下游级联逆运动学解算`,
        type: 'success',
      });

      this.close();
      this.onSaved();
    } catch (err: any) {
      showToast({
        title: '修改失败，已安全回滚',
        message: err.message || '目标无法求解或造成后续关键点超出工作空间',
        type: 'danger',
      });
    }
  }

  close(): void {
    if (this.backdrop && this.backdrop.parentElement) {
      this.backdrop.parentElement.removeChild(this.backdrop);
    }
    this.backdrop = null;
  }
}

function escapeHtml(str: string): string {
  return str.replace(/[&<>'"]/g, tag => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    "'": '&#39;',
    '"': '&quot;',
  }[tag] || tag));
}
