export interface TopBarCallbacks {
  onLoadDemo: () => void;
  onOpenTrajectory: () => void;
  onOpenIO: () => void;
  onToggleTheme: () => void;
  onOpenHelp: () => void;
}

export class TopBar {
  readonly element: HTMLElement;
  private currentTheme: 'light' | 'dark' = 'light';
  private themeBtn: HTMLButtonElement;

  constructor(private readonly callbacks: TopBarCallbacks) {
    this.element = document.createElement('header');
    this.element.className = 'top-bar';
    this.element.innerHTML = `
      <div class="top-bar-left">
        <div class="brand-badge">
          <div class="brand-icon">7R</div>
          <span>SRS 7R 双臂机器人控制台</span>
        </div>
        <div class="status-indicator" title="C++ AdaptiveHybrid IK WebAssembly 引擎已加载">
          <span class="status-dot"></span>
          <span>WASM 引擎在线</span>
        </div>
      </div>

      <div class="top-bar-right">
        <button class="btn btn-secondary btn-sm" id="btn-top-demo" title="载入预置实机动作序列">
          <span>🎬</span>
          <span>载入演示动作</span>
        </button>
        <button class="btn btn-primary btn-sm" id="btn-top-trajectory" title="使用 LaTeX 公式生成关节或笛卡尔函数轨迹">
          <span>ƒ(t)</span>
          <span>函数轨迹</span>
        </button>
        <button class="btn btn-secondary btn-sm" id="btn-top-io" title="导入/导出 performance-action-sequence-v2 JSON">
          <span>💾</span>
          <span>动作序列 IO</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-top-theme" title="切换深浅色彩主题">
          <span id="theme-icon">☀️</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-top-help" title="快捷键与使用指南">
          <span>❓</span>
        </button>
      </div>
    `;

    this.themeBtn = this.element.querySelector('#btn-top-theme') as HTMLButtonElement;

    this.element.querySelector('#btn-top-demo')?.addEventListener('click', () => this.callbacks.onLoadDemo());
    this.element.querySelector('#btn-top-trajectory')?.addEventListener('click', () => this.callbacks.onOpenTrajectory());
    this.element.querySelector('#btn-top-io')?.addEventListener('click', () => this.callbacks.onOpenIO());
    this.themeBtn.addEventListener('click', () => this.callbacks.onToggleTheme());
    this.element.querySelector('#btn-top-help')?.addEventListener('click', () => this.callbacks.onOpenHelp());
  }

  setTheme(theme: 'light' | 'dark'): void {
    this.currentTheme = theme;
    const icon = this.element.querySelector('#theme-icon');
    if (icon) {
      icon.textContent = theme === 'light' ? '☀️' : '🌙';
    }
  }
}
