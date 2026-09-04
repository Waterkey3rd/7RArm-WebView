import { ICONS } from '../icons';

export class HelpDialog {
  private backdrop: HTMLElement | null = null;

  open(): void {
    if (this.backdrop) this.close();

    this.backdrop = document.createElement('div');
    this.backdrop.className = 'modal-backdrop';

    this.backdrop.innerHTML = `
      <div class="modal-window" style="max-width: 680px;">
        <div class="modal-header">
          <div class="modal-title">
            <span class="btn-icon-slot">${ICONS.HELP}</span>
            <span>SRS 7R 机器人工作站使用指南 & 快捷键</span>
          </div>
          <button class="modal-close-btn" id="modal-close-btn">${ICONS.CLOSE}</button>
        </div>

        <div class="modal-body" style="gap: 18px;">
          <div>
            <div style="font-size: 14px; font-weight: 700; margin-bottom: 10px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
              <span>键盘快捷键</span>
            </div>
            <div style="display: grid; grid-template-columns: 140px 1fr; gap: 8px 14px; font-size: 13px;">
              <kbd style="padding: 2px 8px; border: 1px solid var(--border-color); border-radius: var(--radius-sm); font-family: var(--font-mono); background: var(--bg-surface-subtle); text-align: center;">Space</kbd>
              <span>播放 / 暂停动作序列时间轴</span>

              <kbd style="padding: 2px 8px; border: 1px solid var(--border-color); border-radius: var(--radius-sm); font-family: var(--font-mono); background: var(--bg-surface-subtle); text-align: center;">Ctrl + Enter</kbd>
              <span>执行当前控制台设定的运动目标</span>

              <kbd style="padding: 2px 8px; border: 1px solid var(--border-color); border-radius: var(--radius-sm); font-family: var(--font-mono); background: var(--bg-surface-subtle); text-align: center;">Ctrl + Z</kbd>
              <span>回退 / 撤销上一运动状态</span>

              <kbd style="padding: 2px 8px; border: 1px solid var(--border-color); border-radius: var(--radius-sm); font-family: var(--font-mono); background: var(--bg-surface-subtle); text-align: center;">F2</kbd>
              <span>打开当前选中关键点编辑弹窗</span>

              <kbd style="padding: 2px 8px; border: 1px solid var(--border-color); border-radius: var(--radius-sm); font-family: var(--font-mono); background: var(--bg-surface-subtle); text-align: center;">1 / 2 / 3 / 4</kbd>
              <span>快速切换 3D 视角（1 透视、2 顶视、3 正视、4 侧视）</span>

              <kbd style="padding: 2px 8px; border: 1px solid var(--border-color); border-radius: var(--radius-sm); font-family: var(--font-mono); background: var(--bg-surface-subtle); text-align: center;">Esc</kbd>
              <span>关闭打开的模态对话框</span>
            </div>
          </div>

          <hr style="border: none; border-top: 1px solid var(--border-color);">

          <div>
            <div style="font-size: 14px; font-weight: 700; margin-bottom: 8px; color: var(--text-primary);">📐 机器人坐标系与物理单位规范</div>
            <ul style="padding-left: 18px; font-size: 13px; color: var(--text-secondary); line-height: 1.6; display: flex; flex-direction: column; gap: 4px;">
              <li><strong>关节角度 (Joint Angles)</strong>: 界面输入与滑块显示为度 (deg)；WASM 内核与实机导出 JSON 采用规范弧度 (rad)。</li>
              <li><strong>笛卡尔位置 (Cartesian Position)</strong>: X (前向) / Y (左侧) / Z (天向)，单位毫米 (mm)。基准坐标原点位于 J0 机械臂基座。</li>
              <li><strong>笛卡尔姿态 (Orientation)</strong>: 采用 ZYX 欧拉角 (Yaw / Pitch / Roll)，界面输入单位为度 (deg)。</li>
              <li><strong>Delta 局部微调 (Tool Frame Jog)</strong>: 增量矩阵在末端工具坐标系右乘，实现沿工具朝向的精准步进。</li>
              <li><strong>AdaptiveHybrid IK 逆解特性</strong>: 采用 C++ 编译至 WASM 的实机相同算法，具备非对称限位检查与奇异区平滑解算保护。</li>
            </ul>
          </div>
        </div>

        <div class="modal-footer">
          <button class="btn btn-primary" id="btn-help-close">我知道了</button>
        </div>
      </div>
    `;

    document.body.appendChild(this.backdrop);

    this.backdrop.querySelector('#modal-close-btn')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#btn-help-close')?.addEventListener('click', () => this.close());
  }

  close(): void {
    if (this.backdrop && this.backdrop.parentElement) {
      this.backdrop.parentElement.removeChild(this.backdrop);
    }
    this.backdrop = null;
  }
}
