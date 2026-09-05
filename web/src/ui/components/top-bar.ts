import { ICONS } from '../icons';

export interface TopBarCallbacks {
  onLoadDemo: () => void;
  onOpenTrajectory: () => void;
  onOpenHistory: () => void;
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
        <div class="status-indicator" title="C++ AdaptiveHybrid IK WebAssembly 引擎已就绪">
          <span class="status-dot"></span>
          <span>IK 内核在线</span>
        </div>
      </div>

      <div class="top-bar-right">
        <button class="btn btn-secondary btn-sm" id="btn-top-demo" title="载入预置实机动作序列">
          <span class="btn-icon-slot">${ICONS.DEMO}</span>
          <span>载入演示动作</span>
        </button>
        <button class="btn btn-primary btn-sm" id="btn-top-trajectory" title="使用 LaTeX 公式生成关节或笛卡尔函数轨迹">
          <span class="btn-icon-slot">${ICONS.FORMULA}</span>
          <span>函数轨迹</span>
        </button>
        <button class="btn btn-secondary btn-sm" id="btn-top-history" title="查看全部历史操作、关键点和函数来源">
          <span class="btn-icon-slot">${ICONS.HISTORY}</span>
          <span>历史记录</span>
        </button>
        <button class="btn btn-secondary btn-sm" id="btn-top-io" title="导入/导出 performance-action-sequence-v2 JSON">
          <span class="btn-icon-slot">${ICONS.IO}</span>
          <span>动作序列 IO</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-top-theme" title="切换深浅色彩模式">
          <span id="theme-icon">${ICONS.SUN}</span>
        </button>
        <button class="btn btn-ghost btn-icon btn-sm" id="btn-top-help" title="快捷键与使用指南">
          <span>${ICONS.HELP}</span>
        </button>
      </div>
    `;

    this.themeBtn = this.element.querySelector('#btn-top-theme') as HTMLButtonElement;

    this.element.querySelector('#btn-top-demo')?.addEventListener('click', () => this.callbacks.onLoadDemo());
    this.element.querySelector('#btn-top-trajectory')?.addEventListener('click', () => this.callbacks.onOpenTrajectory());
    this.element.querySelector('#btn-top-history')?.addEventListener('click', () => this.callbacks.onOpenHistory());
    this.element.querySelector('#btn-top-io')?.addEventListener('click', () => this.callbacks.onOpenIO());
    this.themeBtn.addEventListener('click', () => this.callbacks.onToggleTheme());
    this.element.querySelector('#btn-top-help')?.addEventListener('click', () => this.callbacks.onOpenHelp());
  }

  setTheme(theme: 'light' | 'dark'): void {
    this.currentTheme = theme;
    const icon = this.element.querySelector('#theme-icon');
    if (icon) {
      icon.innerHTML = theme === 'light' ? ICONS.SUN : ICONS.MOON;
    }
  }
}
