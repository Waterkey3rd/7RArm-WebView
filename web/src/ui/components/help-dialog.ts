import { ICONS } from '../icons';

export class HelpDialog {
  private backdrop: HTMLElement | null = null;
  private currentTab = 0;
  private readonly tabs = [
    { id: 'quickstart', label: '快速上手', icon: ICONS.TARGET },
    { id: 'viewport', label: '3D 视口导航', icon: ICONS.MOUSE },
    { id: 'modes', label: '运动控制模式', icon: ICONS.SLIDERS },
    { id: 'timeline', label: '时间轴与功能', icon: ICONS.CLOCK },
    { id: 'shortcuts', label: '快捷键速查', icon: ICONS.KEYBOARD },
  ];

  open(): void {
    if (this.backdrop) this.close();

    this.currentTab = 0;
    this.backdrop = document.createElement('div');
    this.backdrop.className = 'modal-backdrop';

    this.backdrop.innerHTML = `
      <div class="modal-window guide-modal-window">
        <!-- Header -->
        <div class="modal-header">
          <div class="modal-title">
            <span class="btn-icon-slot">${ICONS.HELP}</span>
            <span>SRS 7R 双机械臂工作台 · 操作使用指南</span>
          </div>
          <button class="modal-close-btn" id="modal-close-btn" title="关闭指南 (Esc)">${ICONS.CLOSE}</button>
        </div>

        <!-- Navigation Tabs -->
        <div class="guide-nav-bar">
          <div class="segmented-nav" id="guide-segmented-nav">
            ${this.tabs
              .map(
                (tab, idx) => `
              <button class="segment-btn ${idx === 0 ? 'active' : ''}" data-tab-idx="${idx}">
                <span class="btn-icon-slot">${tab.icon}</span>
                <span>${tab.label}</span>
              </button>
            `,
              )
              .join('')}
          </div>
        </div>

        <!-- Body Content Area -->
        <div class="modal-body guide-modal-body" id="guide-modal-content">
          ${this.renderTabContent(0)}
        </div>

        <!-- Footer -->
        <div class="modal-footer" style="justify-content: space-between;">
          <label class="guide-chk-label">
            <input type="checkbox" id="chk-guide-dont-show">
            <span>下次进入不再自动弹出</span>
          </label>
          <div style="display: flex; gap: 8px;">
            <button class="btn btn-secondary" id="btn-guide-prev" style="display: none;">上一页</button>
            <button class="btn btn-secondary" id="btn-guide-next">下一页</button>
            <button class="btn btn-primary" id="btn-help-close">开始使用</button>
          </div>
        </div>
      </div>
    `;

    document.body.appendChild(this.backdrop);

    // Initial checkbox state from localStorage
    const chk = this.backdrop.querySelector('#chk-guide-dont-show') as HTMLInputElement;
    if (chk) {
      chk.checked = localStorage.getItem('srs_7r_guide_viewed') === 'true';
    }

    // Bind tab switching
    this.backdrop.querySelectorAll<HTMLButtonElement>('#guide-segmented-nav .segment-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const idx = parseInt(btn.dataset.tabIdx || '0', 10);
        this.switchTab(idx);
      });
    });

    // Bind footer buttons
    this.backdrop.querySelector('#modal-close-btn')?.addEventListener('click', () => this.saveAndClose());
    this.backdrop.querySelector('#btn-help-close')?.addEventListener('click', () => this.saveAndClose());

    this.backdrop.querySelector('#btn-guide-prev')?.addEventListener('click', () => {
      if (this.currentTab > 0) this.switchTab(this.currentTab - 1);
    });

    this.backdrop.querySelector('#btn-guide-next')?.addEventListener('click', () => {
      if (this.currentTab < this.tabs.length - 1) {
        this.switchTab(this.currentTab + 1);
      } else {
        this.saveAndClose();
      }
    });

    // Close on backdrop click (click outside window)
    this.backdrop.addEventListener('click', (e) => {
      if (e.target === this.backdrop) {
        this.saveAndClose();
      }
    });
  }

  private switchTab(index: number): void {
    if (!this.backdrop) return;
    this.currentTab = index;

    // Update tab buttons
    this.backdrop.querySelectorAll<HTMLButtonElement>('#guide-segmented-nav .segment-btn').forEach((btn, idx) => {
      btn.classList.toggle('active', idx === index);
    });

    // Update body
    const contentArea = this.backdrop.querySelector('#guide-modal-content') as HTMLElement;
    if (contentArea) {
      contentArea.innerHTML = this.renderTabContent(index);
      contentArea.scrollTop = 0;
    }

    // Update Prev / Next buttons
    const prevBtn = this.backdrop.querySelector('#btn-guide-prev') as HTMLElement;
    const nextBtn = this.backdrop.querySelector('#btn-guide-next') as HTMLElement;
    if (prevBtn) prevBtn.style.display = index === 0 ? 'none' : 'inline-flex';
    if (nextBtn) nextBtn.style.display = index === this.tabs.length - 1 ? 'none' : 'inline-flex';
  }

  private renderTabContent(index: number): string {
    switch (index) {
      case 0:
        return this.renderQuickStart();
      case 1:
        return this.renderViewportGuide();
      case 2:
        return this.renderControlModesGuide();
      case 3:
        return this.renderTimelineAndFeatures();
      case 4:
        return this.renderShortcutsAndSpecs();
      default:
        return '';
    }
  }

  // 1. Quick Start
  private renderQuickStart(): string {
    return `
      <div class="guide-intro-banner">
        <div class="guide-intro-title">
          <span class="btn-icon-slot" style="margin-right: 6px;">${ICONS.BRAND}</span>
          <span>欢迎使用 SRS 7R 双机械臂可视化与动作序列编排系统</span>
        </div>
        <div class="guide-intro-desc">
          基于 WebAssembly C++ AdaptiveHybrid IK 逆解内核与 Three.js 物理拟真渲染。您可以在此进行双臂正逆运动学解算、多关键点动作示教编排、复杂轨迹自动生成与实机代码导出。
        </div>
      </div>

      <div style="display: flex; flex-direction: column; gap: 12px; margin-top: 4px;">
        <div class="guide-step-card">
          <div class="guide-step-num">1</div>
          <div class="guide-step-content">
            <div class="guide-step-heading">
              <span class="btn-icon-slot" style="margin-right: 4px;">${ICONS.SPARKLES}</span>
              <span>一键体验：载入预设 Demo 动作序列</span>
            </div>
            <div class="guide-step-text">
              初次使用？点击顶部导航栏右上角的 <strong>「载入演示动作」</strong> 按钮，系统将自动注入预设的双臂协同律动动作序列，并可在底部时间轴即刻播放体验！
            </div>
          </div>
        </div>

        <div class="guide-step-card">
          <div class="guide-step-num">2</div>
          <div class="guide-step-content">
            <div class="guide-step-heading">
              <span class="btn-icon-slot" style="margin-right: 4px;">${ICONS.MOUSE}</span>
              <span>视口观察：Blender 规范视角交互</span>
            </div>
            <div class="guide-step-text">
              按住 <strong>鼠标中键</strong> 拖拽自由旋转视角，<strong>Shift + 鼠标中键</strong> 平移视野，<strong>鼠标滚轮</strong> 或 <strong>Ctrl + 鼠标中键</strong> 缩放。笔记本用户支持 <strong>Alt + 鼠标左键</strong> 模拟三键鼠标。
            </div>
          </div>
        </div>

        <div class="guide-step-card">
          <div class="guide-step-num">3</div>
          <div class="guide-step-content">
            <div class="guide-step-heading">
              <span class="btn-icon-slot" style="margin-right: 4px;">${ICONS.SLIDERS}</span>
              <span>控制与编排：设定姿态并记录时间轴</span>
            </div>
            <div class="guide-step-text">
              在右侧控制台切换 <strong>关节角</strong>、<strong>笛卡尔空间</strong> 或 <strong>Delta 微调</strong> 模式。调整数值后点击「执行运动」（或按 <kbd>Ctrl+Enter</kbd>），机械臂平滑运动并自动生成时间轴关键点。
            </div>
          </div>
        </div>

        <div class="guide-step-card">
          <div class="guide-step-num">4</div>
          <div class="guide-step-content">
            <div class="guide-step-heading">
              <span class="btn-icon-slot" style="margin-right: 4px;">${ICONS.CLOCK}</span>
              <span>序列回放与导出：时间轴与 I/O 互通</span>
            </div>
            <div class="guide-step-text">
              按 <kbd>Space</kbd> 随时播放动作序列，支持单帧步进、循环、倍速调节。通过顶栏「动作序列 IO」可将编辑好的姿态序列保存为 JSON 文件，或下发至实机控制器运行。
            </div>
          </div>
        </div>
      </div>
    `;
  }

  // 2. Viewport Guide (Blender Navigation)
  private renderViewportGuide(): string {
    return `
      <div>
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 8px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.MOUSE}</span>
          <span>3D 视口导航操作规范 (符合 Blender 习惯)</span>
        </div>
        <div style="font-size: 13px; color: var(--text-secondary); margin-bottom: 14px; line-height: 1.6;">
          系统视口控制已全面对齐专业三维软件 <strong>Blender</strong> 交互逻辑。常规左键点击不会误触镜头旋转，方便您精准选定与操作。
        </div>

        <div class="guide-table-grid">
          <div class="guide-grid-header">操作意图</div>
          <div class="guide-grid-header">标准外接鼠标操作</div>
          <div class="guide-grid-header">笔记本触控板 (三键模拟)</div>

          <div class="guide-grid-cell font-bold">视角旋转 (Orbit)</div>
          <div class="guide-grid-cell">
            <kbd>鼠标中键</kbd> 拖拽 (按下滚轮拖拽)
          </div>
          <div class="guide-grid-cell">
            <kbd>Alt</kbd> + <kbd>左键</kbd> 拖拽
          </div>

          <div class="guide-grid-cell font-bold">视野平移 (Pan)</div>
          <div class="guide-grid-cell">
            <kbd>Shift</kbd> + <kbd>鼠标中键</kbd> 拖拽 (或右键拖拽)
          </div>
          <div class="guide-grid-cell">
            <kbd>Alt</kbd> + <kbd>Shift</kbd> + <kbd>左键</kbd> 拖拽
          </div>

          <div class="guide-grid-cell font-bold">视角推拉缩放 (Zoom)</div>
          <div class="guide-grid-cell">
            滚动 <kbd>鼠标滚轮</kbd> 或 <kbd>Ctrl</kbd> + <kbd>中键</kbd> 拖拽
          </div>
          <div class="guide-grid-cell">
            <kbd>Alt</kbd> + <kbd>Ctrl</kbd> + <kbd>左键</kbd> 拖拽 / 双指捏合
          </div>
        </div>
      </div>

      <div style="margin-top: 14px;">
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 8px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.CUBE}</span>
          <span>快速预设视角与视口图层工具栏</span>
        </div>
        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 12px;">
          <div class="guide-info-card">
            <div style="font-weight: 700; color: var(--text-primary); margin-bottom: 4px;">快捷数字键视角切换</div>
            <ul style="padding-left: 16px; color: var(--text-secondary); line-height: 1.8;">
              <li><kbd>1</kbd>：等轴透视视角 (默认全局视角)</li>
              <li><kbd>2</kbd>：正上方顶视视角 (Top)</li>
              <li><kbd>3</kbd>：正前方正视视角 (Front)</li>
              <li><kbd>4</kbd>：右侧方侧视视角 (Side)</li>
            </ul>
          </div>
          <div class="guide-info-card">
            <div style="font-weight: 700; color: var(--text-primary); margin-bottom: 4px;">视口左下角辅助图层开关</div>
            <ul style="padding-left: 16px; color: var(--text-secondary); line-height: 1.8;">
              <li><strong>复位视角</strong>：相机平滑过渡回初始等轴透视位</li>
              <li><strong>网格开关</strong>：显示/隐藏 1200mm 地面参考网格</li>
              <li><strong>关节轴开关</strong>：显示 7 个旋转关节的旋转轴向量</li>
              <li><strong>末端系开关</strong>：显示末端执行器 RGB 局部坐标系</li>
            </ul>
          </div>
        </div>
      </div>
    `;
  }

  // 3. Control Modes Guide
  private renderControlModesGuide(): string {
    return `
      <div>
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 8px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.SLIDERS}</span>
          <span>右侧控制台：三大运动控制模式</span>
        </div>
        <div style="font-size: 13px; color: var(--text-secondary); margin-bottom: 12px; line-height: 1.5;">
          控制台支持在 <strong>关节角 (Joint)</strong>、<strong>笛卡尔 (Cartesian)</strong> 和 <strong>Delta 微调</strong> 之间无缝切换：
        </div>

        <div style="display: flex; flex-direction: column; gap: 12px;">
          <!-- Mode 1 -->
          <div class="guide-mode-box">
            <div class="guide-mode-header">
              <span class="guide-mode-badge" style="background: var(--bg-surface-subtle); color: var(--text-primary); border: 1px solid var(--border-color);">模式 1</span>
              <span style="font-weight: 700;">关节空间控制 (Joint Space)</span>
            </div>
            <div class="guide-mode-desc">
              直接驱动左右臂各自的 7 个旋转关节角度（度 deg）。包含安全限位检测指示条，超出极限自动警告。
              <div style="margin-top: 6px; font-size: 12px; color: var(--text-muted); display: flex; align-items: center; gap: 4px;">
                <span class="btn-icon-slot">${ICONS.LIGHTBULB}</span>
                <span>预设工具栏提供 <strong>零位姿态</strong> (所有角度归零)、<strong>准备姿态</strong> (预就绪构型) 及 <strong>左向右镜像</strong> (左右对称复制)。</span>
              </div>
            </div>
          </div>

          <!-- Mode 2 -->
          <div class="guide-mode-box">
            <div class="guide-mode-header">
              <span class="guide-mode-badge" style="background: var(--bg-surface-subtle); color: var(--text-primary); border: 1px solid var(--border-color);">模式 2</span>
              <span style="font-weight: 700;">末端笛卡尔位姿控制 (Cartesian Space)</span>
            </div>
            <div class="guide-mode-desc">
              输入末端工具目标坐标：位置 <strong>X, Y, Z (毫米 mm)</strong> 与欧拉姿态 <strong>Yaw, Pitch, Roll (度 deg)</strong>。
              <div style="margin-top: 6px; font-size: 12px; color: var(--text-muted); display: flex; align-items: center; gap: 4px;">
                <span class="btn-icon-slot">${ICONS.LIGHTBULB}</span>
                <span>内置实时 <strong>AdaptiveHybrid IK 逆解探测器</strong>：绿色徽标代表解算可达且显示空间误差；红色代表超出工作空间或姿态奇异。</span>
              </div>
            </div>
          </div>

          <!-- Mode 3 -->
          <div class="guide-mode-box">
            <div class="guide-mode-header">
              <span class="guide-mode-badge" style="background: var(--bg-surface-subtle); color: var(--text-primary); border: 1px solid var(--border-color);">模式 3</span>
              <span style="font-weight: 700;">Delta 工具坐标系微调 (Tool Frame Jog)</span>
            </div>
            <div class="guide-mode-desc">
              用于对机械臂末端进行高精度局部调整。支持选择步进跨度 (1mm/1° ~ 25mm/25°)。
              <div style="margin-top: 6px; font-size: 12px; color: var(--text-muted); display: flex; align-items: center; gap: 4px;">
                <span class="btn-icon-slot">${ICONS.LIGHTBULB}</span>
                <span><strong>即点即动</strong>：点击 +X/-X/+Y/-Y/+Z/-Z 或 +Yaw/-Yaw/+Pitch/-Pitch/+Roll/-Roll 按键，机械臂即时执行微步进，无需再次点击执行按钮！顶部设有「左臂 / 右臂 / 双臂」切换标签，布局紧凑不遮挡。</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    `;
  }

  // 4. Timeline & Advanced Features
  private renderTimelineAndFeatures(): string {
    return `
      <div>
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 8px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.CLOCK}</span>
          <span>底部时间轴动作序列编排系统</span>
        </div>
        <div style="font-size: 13px; color: var(--text-secondary); margin-bottom: 12px; line-height: 1.5;">
          任何一次姿态运动执行完毕后，系统都会自动记录为时间轴上的一帧关键点 (Keypoint)。
        </div>

        <div class="guide-features-grid">
          <div class="guide-feature-item">
            <div class="guide-feature-title" style="display: flex; align-items: center; gap: 6px;">
              <span class="btn-icon-slot">${ICONS.PLAY}</span>
              <span>播放与步进控制</span>
            </div>
            <div class="guide-feature-desc">
              按 <kbd>Space</kbd> 播放或暂停时间轴。支持上一帧/下一帧单步跳跃、首尾帧快速直达。
            </div>
          </div>

          <div class="guide-feature-item">
            <div class="guide-feature-title" style="display: flex; align-items: center; gap: 6px;">
              <span class="btn-icon-slot">${ICONS.LOOP}</span>
              <span>循环与倍速回放</span>
            </div>
            <div class="guide-feature-desc">
              可勾选循环播放模式，支持 0.5x、1.0x、1.5x、2.0x、4.0x 多档运动播放速率无级变速。
            </div>
          </div>

          <div class="guide-feature-item">
            <div class="guide-feature-title" style="display: flex; align-items: center; gap: 6px;">
              <span class="btn-icon-slot">${ICONS.EDIT}</span>
              <span>序列卡片交互与编辑</span>
            </div>
            <div class="guide-feature-desc">
              点击时间轴任意卡片可直接使机器人跳转至该位姿；点击卡片右侧菜单或按 <kbd>F2</kbd> 可重命名并编辑关节角度。
            </div>
          </div>

          <div class="guide-feature-item">
            <div class="guide-feature-title" style="display: flex; align-items: center; gap: 6px;">
              <span class="btn-icon-slot">${ICONS.FORMULA}</span>
              <span>轨迹自动生成器 (Trajectory)</span>
            </div>
            <div class="guide-feature-desc">
              顶栏「函数轨迹」支持根据数学解析式自动离散生成<strong>空间直线</strong>、<strong>圆弧轨迹</strong>、<strong>抛物线</strong>及<strong>避障路径</strong>，直接插入动作序列。
            </div>
          </div>
        </div>
      </div>

      <div style="margin-top: 14px;">
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 6px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.IO}</span>
          <span>动作序列导入导出与操作日志</span>
        </div>
        <div style="font-size: 13px; color: var(--text-secondary); line-height: 1.6;">
          • <strong>动作序列 IO</strong>：可将编排好的动作时间轴导出为规范的 JSON 文件备份或共享，也可一键载入外部动作配置。<br>
          • <strong>历史记录 (Log)</strong>：记录系统每次正逆解计算耗时、位姿误差及用户修改流水账，可追溯定位历史姿态。
        </div>
      </div>
    `;
  }

  // 5. Shortcuts & Specifications
  private renderShortcutsAndSpecs(): string {
    return `
      <div>
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 10px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.KEYBOARD}</span>
          <span>全局键盘快捷键一览</span>
        </div>

        <div style="display: grid; grid-template-columns: 140px 1fr; gap: 8px 14px; font-size: 13px;">
          <kbd class="guide-kbd">Space</kbd>
          <span>播放 / 暂停动作序列时间轴回放</span>

          <kbd class="guide-kbd">Ctrl + Enter</kbd>
          <span>执行当前控制台设定的运动目标 (关节角 / 笛卡尔模式)</span>

          <kbd class="guide-kbd">Ctrl + Z</kbd>
          <span>回退 / 撤销上一运动状态</span>

          <kbd class="guide-kbd">F2</kbd>
          <span>打开当前选中关键点编辑与详情弹窗</span>

          <kbd class="guide-kbd">1 / 2 / 3 / 4</kbd>
          <span>快速切换 3D 视角（1 透视、2 顶视、3 正视、4 侧视）</span>

          <kbd class="guide-kbd">Esc</kbd>
          <span>关闭当前打开的弹窗 / 浮层</span>
        </div>
      </div>

      <hr style="border: none; border-top: 1px solid var(--border-color); margin: 16px 0;">

      <div>
        <div style="font-size: 14px; font-weight: 700; margin-bottom: 8px; color: var(--text-primary); display: flex; align-items: center; gap: 6px;">
          <span class="btn-icon-slot">${ICONS.RULER}</span>
          <span>机器人坐标系与物理单位规范</span>
        </div>
        <ul style="padding-left: 18px; font-size: 13px; color: var(--text-secondary); line-height: 1.6; display: flex; flex-direction: column; gap: 6px;">
          <li><strong>关节角度 (Joint Angles)</strong>: 界面输入与滑块显示为度 (deg)；WASM 内核与实机导出 JSON 采用国际规范弧度 (rad)。</li>
          <li><strong>笛卡尔位置 (Cartesian Position)</strong>: X (前向) / Y (左侧) / Z (天向)，单位毫米 (mm)。基准坐标原点位于 J0 机械臂基座。</li>
          <li><strong>笛卡尔姿态 (Orientation)</strong>: 采用 ZYX 欧拉角 (Yaw / Pitch / Roll)，界面输入单位为度 (deg)。</li>
          <li><strong>Delta 局部微调 (Tool Frame Jog)</strong>: 增量矩阵在末端工具坐标系右乘，实现沿工具朝向的精准相对步进。</li>
          <li><strong>AdaptiveHybrid IK 逆解特性</strong>: 采用 C++ 编译至 WASM 的实机相同算法，具备非对称限位检查与奇异区平滑解算保护。</li>
        </ul>
      </div>
    `;
  }

  private saveAndClose(): void {
    if (!this.backdrop) return;
    const chk = this.backdrop.querySelector('#chk-guide-dont-show') as HTMLInputElement;
    if (chk && chk.checked) {
      localStorage.setItem('srs_7r_guide_viewed', 'true');
    } else {
      localStorage.removeItem('srs_7r_guide_viewed');
    }
    this.close();
  }

  close(): void {
    if (this.backdrop && this.backdrop.parentElement) {
      this.backdrop.parentElement.removeChild(this.backdrop);
    }
    this.backdrop = null;
  }
}

