import { RoboArmController } from '../controller';
import { ArmRenderer } from '../renderer';
import type { ArmState, Side } from '../types';
import { cloneState, SIDES } from '../types';
import { ControlPanel } from './components/control-panel';
import { HelpDialog } from './components/help-dialog';
import { HistoryDialog } from './components/history-dialog';
import { IoDialog } from './components/io-dialog';
import { TimelineDock } from './components/timeline';
import { showToast } from './components/toast';
import { TopBar } from './components/top-bar';
import { TrajectoryDialog } from './components/trajectory-dialog';
import { ViewportHud } from './components/viewport-hud';

export class App {
  private renderer!: ArmRenderer;
  private topBar!: TopBar;
  private viewportHud!: ViewportHud;
  private controlPanel!: ControlPanel;
  private timeline!: TimelineDock;

  private trajectoryDialog!: TrajectoryDialog;
  private historyDialog!: HistoryDialog;
  private ioDialog!: IoDialog;
  private helpDialog!: HelpDialog;

  private currentTheme: 'light' | 'dark' = 'light';
  private activeKeypointIndex = -1;

  // Animation & Playback state
  private isAnimating = false;
  private isPlaying = false;
  private isLooping = false;
  private playbackSpeed = 1.0;
  private currentPlaybackIndex = 0;
  private playbackStartTime = 0;
  private animationRafId = 0;

  private constructor(
    private readonly host: HTMLElement,
    private readonly controller: RoboArmController,
  ) {}

  static async init(host: HTMLElement, wasmBase = './wasm/'): Promise<App> {
    const controller = await RoboArmController.create(wasmBase);
    const app = new App(host, controller);
    app.setup();
    return app;
  }

  private setup(): void {
    // Root container
    this.host.innerHTML = '';
    const root = document.createElement('div');
    root.className = 'app-root';
    this.host.appendChild(root);

    // Setup TopBar
    this.topBar = new TopBar({
      onLoadDemo: () => this.loadDemoSequence(),
      onOpenTrajectory: () => this.trajectoryDialog.open(),
      onOpenIO: () => this.ioDialog.open(),
      onToggleTheme: () => this.toggleTheme(),
      onOpenHelp: () => this.helpDialog.open(),
    });
    root.appendChild(this.topBar.element);

    // Workspace Body
    const workspace = document.createElement('div');
    workspace.className = 'workspace-body';
    root.appendChild(workspace);

    // 3D Viewport
    const viewportContainer = document.createElement('div');
    viewportContainer.className = 'viewport-container';
    const canvasHost = document.createElement('div');
    canvasHost.className = 'viewport-canvas-host';
    viewportContainer.appendChild(canvasHost);
    workspace.appendChild(viewportContainer);

    this.renderer = new ArmRenderer(canvasHost, this.controller.ik, {
      theme: this.currentTheme,
      showGrid: true,
      showAxes: true,
      showTool: true,
    });

    // Viewport HUD
    this.viewportHud = new ViewportHud(viewportContainer, this.controller.ik, {
      onSelectCamera: (preset) => this.renderer.setCameraPreset(preset, true),
      onResetCamera: () => this.renderer.resetCamera(),
      onToggleGrid: () => this.renderer.toggleGrid(),
      onToggleAxes: () => this.renderer.toggleAxes(),
      onToggleTool: () => this.renderer.toggleTool(),
    });

    // Right Control Dock
    this.controlPanel = new ControlPanel(workspace, this.controller, {
      onExecuteJoint: (values, duration) => this.executeJoint(values, duration),
      onExecuteCartesian: (values, duration) => this.executeCartesian(values, duration),
      onExecuteDelta: (values, duration) => this.executeDelta(values, duration),
      onUndo: () => this.undo(),
      onReset: () => this.reset(),
    });

    // Bottom Timeline Dock
    this.timeline = new TimelineDock(root, this.controller, {
      onPlayPause: () => this.togglePlayPause(),
      onStepFrame: (dir) => this.stepFrame(dir),
      onGoToStart: () => this.goToKeypoint(0),
      onGoToEnd: () => this.goToKeypoint(this.controller.history.length - 1),
      onToggleLoop: (loop) => { this.isLooping = loop; },
      onChangeSpeed: (speed) => { this.playbackSpeed = speed; },
      onScrub: (progress) => this.scrubTo(progress),
      onSelectKeypoint: (index) => this.goToKeypoint(index),
      onEditKeypoint: (index) => this.historyDialog.open(index),
      onDeleteKeypoint: (index) => this.deleteKeypoint(index),
    });

    // Dialogs
    this.trajectoryDialog = new TrajectoryDialog(this.controller, (generated) => {
      const firstGeneratedIndex = this.controller.history.length - generated.length;
      this.stopPlayback();
      this.isAnimating = false;
      this.activeKeypointIndex = this.controller.history.length - 1;
      this.timeline.updateTimeline(this.activeKeypointIndex);
      if (generated.length > 0) {
        this.renderer.update(generated[0].start);
        this.viewportHud.update(generated[0].start);
        this.controlPanel.syncState(generated[0].start);
        this.startPlayback(firstGeneratedIndex);
      }
    });

    this.historyDialog = new HistoryDialog(this.controller, () => {
      this.timeline.updateTimeline(this.activeKeypointIndex);
      this.syncAllViews();
    });

    this.ioDialog = new IoDialog(this.controller, () => {
      this.activeKeypointIndex = this.controller.history.length ? 0 : -1;
      this.timeline.updateTimeline(this.activeKeypointIndex);
      this.syncAllViews();
    });

    this.helpDialog = new HelpDialog();

    // Subscribe to controller state changes
    this.controller.subscribe(() => {
      this.syncAllViews();
    });

    // Initial render sync
    this.syncAllViews();

    // Bind Keyboard shortcuts
    this.bindKeyboardShortcuts();
  }

  private syncAllViews(): void {
    const cur = this.controller.current;
    this.renderer.update(cur);
    this.viewportHud.update(cur);
    this.controlPanel.syncState(cur);
  }

  private toggleTheme(): void {
    this.currentTheme = this.currentTheme === 'light' ? 'dark' : 'light';
    document.documentElement.setAttribute('data-theme', this.currentTheme);
    this.renderer.setTheme(this.currentTheme);
    this.topBar.setTheme(this.currentTheme);
    showToast({ message: `已切换为${this.currentTheme === 'light' ? '浅色明亮实验室' : '深色座舱'}主题`, type: 'info' });
  }

  // =========================================================
  // Motion Execution with Smooth Interpolation
  // =========================================================
  private executeJoint(values: Record<Side, number[]>, durationMs: number): void {
    if (this.isAnimating) return;
    try {
      const prev = cloneState(this.controller.current);
      const point = this.controller.commandJoints(values, { durationMs });
      this.animateTransition(prev, point.target, durationMs, () => {
        this.activeKeypointIndex = this.controller.history.length - 1;
        this.timeline.updateTimeline(this.activeKeypointIndex);
      });
      showToast({ message: '关节运动执行完成', type: 'success' });
    } catch (err: any) {
      showToast({ title: '执行失败', message: err.message || '指令超出关节限位', type: 'danger' });
    }
  }

  private executeCartesian(values: Record<Side, number[]>, durationMs: number): void {
    if (this.isAnimating) return;
    try {
      const prev = cloneState(this.controller.current);
      const point = this.controller.commandCartesian(values, { durationMs });
      this.animateTransition(prev, point.target, durationMs, () => {
        this.activeKeypointIndex = this.controller.history.length - 1;
        this.timeline.updateTimeline(this.activeKeypointIndex);
      });
      showToast({ message: '笛卡尔空间运动执行完成', type: 'success' });
    } catch (err: any) {
      showToast({ title: '逆解失败', message: err.message || '目标超出工作空间', type: 'danger' });
    }
  }

  private executeDelta(values: Record<Side, number[]>, durationMs: number): void {
    if (this.isAnimating) return;
    try {
      const prev = cloneState(this.controller.current);
      const point = this.controller.commandDelta(values, { durationMs });
      this.animateTransition(prev, point.target, durationMs, () => {
        this.activeKeypointIndex = this.controller.history.length - 1;
        this.timeline.updateTimeline(this.activeKeypointIndex);
      });
    } catch (err: any) {
      showToast({ title: '微调失败', message: err.message || '微调增量超出机械臂极限', type: 'danger' });
    }
  }

  private animateTransition(
    from: ArmState,
    to: ArmState,
    durationMs: number,
    onComplete?: () => void,
  ): void {
    this.isAnimating = true;
    const start = performance.now();

    const frame = {
      start: cloneState(from),
      target: cloneState(to),
      durationMs,
      historyIndex: 0,
    };

    const tick = (now: number) => {
      const elapsed = (now - start) * this.playbackSpeed;
      const progress = Math.min(1, elapsed / durationMs);

      const interpolated = this.controller.interpolate(frame, elapsed);
      this.renderer.update(interpolated);
      this.viewportHud.update(interpolated);
      this.viewportHud.showMotionProgress(true, `运动过渡中... ${(progress * 100).toFixed(0)}%`);

      if (progress < 1) {
        this.animationRafId = requestAnimationFrame(tick);
      } else {
        this.isAnimating = false;
        this.viewportHud.showMotionProgress(false);
        this.syncAllViews();
        if (onComplete) onComplete();
      }
    };

    this.animationRafId = requestAnimationFrame(tick);
  }

  private undo(): void {
    if (this.isAnimating) return;
    if (this.controller.history.length === 0) {
      showToast({ message: '当前已在初始位姿，无法继续回退', type: 'info' });
      return;
    }
    this.controller.undo();
    this.activeKeypointIndex = this.controller.history.length - 1;
    this.timeline.updateTimeline(this.activeKeypointIndex);
    this.syncAllViews();
    showToast({ message: '已撤销回退至上一状态', type: 'info' });
  }

  private reset(): void {
    if (this.isAnimating) return;
    this.stopPlayback();
    this.controller.reset();
    this.activeKeypointIndex = -1;
    this.timeline.updateTimeline(-1);
    this.syncAllViews();
    showToast({ message: '机械臂已重置为初始零位姿态', type: 'info' });
  }

  // =========================================================
  // Timeline Playback Engine
  // =========================================================
  private togglePlayPause(): void {
    if (this.isPlaying) {
      this.stopPlayback();
    } else {
      this.startPlayback();
    }
  }

  private startPlayback(fromIndex = 0): void {
    if (this.controller.history.length === 0) {
      showToast({ message: '动作序列为空，无法播放', type: 'warning' });
      return;
    }

    this.isPlaying = true;
    this.timeline.setPlayState(true);
    this.currentPlaybackIndex = Math.max(0, Math.min(this.controller.history.length - 1, fromIndex));
    this.playbackStartTime = performance.now();
    this.playNextFrame();
  }

  private stopPlayback(): void {
    this.isPlaying = false;
    this.timeline.setPlayState(false);
    this.viewportHud.showMotionProgress(false);
    cancelAnimationFrame(this.animationRafId);
  }

  private playNextFrame(): void {
    if (!this.isPlaying) return;

    const history = this.controller.history;
    if (this.currentPlaybackIndex >= history.length) {
      if (this.isLooping && history.length > 0) {
        this.currentPlaybackIndex = 0;
      } else {
        this.stopPlayback();
        showToast({ message: '动作序列播放完毕', type: 'success' });
        return;
      }
    }

    const point = history[this.currentPlaybackIndex];
    this.activeKeypointIndex = this.currentPlaybackIndex;
    this.timeline.updateTimeline(this.currentPlaybackIndex);

    const frame = {
      start: cloneState(point.start),
      target: cloneState(point.target),
      durationMs: point.durationMs,
      historyIndex: this.currentPlaybackIndex,
    };

    const start = performance.now();
    const duration = point.durationMs;

    const tick = (now: number) => {
      if (!this.isPlaying) return;
      const elapsed = (now - start) * this.playbackSpeed;
      const progress = Math.min(1, elapsed / duration);

      const interpolated = this.controller.interpolate(frame, elapsed);
      this.renderer.update(interpolated);
      this.viewportHud.update(interpolated);

      // Calculate total sequence progress
      let elapsedTotalMs = 0;
      let sequenceTotalMs = 0;
      for (let i = 0; i < history.length; i++) {
        if (i < this.currentPlaybackIndex) elapsedTotalMs += history[i].durationMs;
        sequenceTotalMs += history[i].durationMs;
      }
      elapsedTotalMs += progress * duration;
      const totalRatio = sequenceTotalMs > 0 ? elapsedTotalMs / sequenceTotalMs : 0;
      this.timeline.setProgress(totalRatio, elapsedTotalMs / 1000, sequenceTotalMs / 1000);

      this.viewportHud.showMotionProgress(
        true,
        `播放关键点 #${this.currentPlaybackIndex + 1}/${history.length}: ${point.label} (${(progress * 100).toFixed(0)}%)`,
      );

      if (progress < 1) {
        this.animationRafId = requestAnimationFrame(tick);
      } else {
        this.currentPlaybackIndex++;
        this.playNextFrame();
      }
    };

    this.animationRafId = requestAnimationFrame(tick);
  }

  private stepFrame(direction: -1 | 1): void {
    this.stopPlayback();
    const history = this.controller.history;
    if (history.length === 0) return;

    let next = this.activeKeypointIndex + direction;
    next = Math.max(0, Math.min(history.length - 1, next));
    this.goToKeypoint(next);
  }

  private goToKeypoint(index: number): void {
    this.stopPlayback();
    const history = this.controller.history;
    if (index < 0 || index >= history.length) return;

    this.activeKeypointIndex = index;
    this.controller.restore(index);
    this.timeline.updateTimeline(index);
    this.syncAllViews();
  }

  private scrubTo(progress: number): void {
    this.stopPlayback();
    const history = this.controller.history;
    if (history.length === 0) return;

    let totalDurationMs = 0;
    history.forEach(p => totalDurationMs += p.durationMs);
    const targetMs = progress * totalDurationMs;

    let accumulated = 0;
    for (let i = 0; i < history.length; i++) {
      const dur = history[i].durationMs;
      if (accumulated + dur >= targetMs || i === history.length - 1) {
        const frameProgress = Math.max(0, Math.min(1, (targetMs - accumulated) / dur));
        const frame = {
          start: cloneState(history[i].start),
          target: cloneState(history[i].target),
          durationMs: dur,
          historyIndex: i,
        };
        const state = this.controller.interpolate(frame, frameProgress * dur);
        this.renderer.update(state);
        this.viewportHud.update(state);
        this.controlPanel.syncState(state);
        this.timeline.setProgress(progress, targetMs / 1000, totalDurationMs / 1000);
        this.activeKeypointIndex = i;
        break;
      }
      accumulated += dur;
    }
  }

  private deleteKeypoint(index: number): void {
    if (this.isAnimating) return;
    this.stopPlayback();
    if (this.controller.history.length <= 1) {
      this.reset();
      return;
    }

    try {
      this.controller.deleteHistory(index);
      this.activeKeypointIndex = Math.min(this.activeKeypointIndex, this.controller.history.length - 1);
      this.timeline.updateTimeline(this.activeKeypointIndex);
      this.syncAllViews();
      showToast({ message: `已删除关键点 #${index + 1}`, type: 'info' });
    } catch (err: any) {
      showToast({ title: '删除失败', message: err.message || '删除该点后下游关键点无有效解', type: 'danger' });
    }
  }

  // =========================================================
  // Demo Sequence Loader
  // =========================================================
  private async loadDemoSequence(): Promise<void> {
    try {
      const res = await fetch('./demo_dance.json');
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const payload = await res.json();
      this.controller.import(payload);
      this.activeKeypointIndex = 0;
      this.timeline.updateTimeline(0);
      this.syncAllViews();
      showToast({
        title: '演示动作载入成功',
        message: `成功载入内置实机舞蹈动作（共 ${this.controller.history.length} 个关键点），点击播放即可演示`,
        type: 'success',
      });
    } catch (err: any) {
      showToast({
        title: '载入失败',
        message: '无法读取 demo_dance.json: ' + (err.message || String(err)),
        type: 'danger',
      });
    }
  }

  // =========================================================
  // Keyboard Shortcuts
  // =========================================================
  private bindKeyboardShortcuts(): void {
    window.addEventListener('keydown', (e: KeyboardEvent) => {
      // If typing in input or textarea, skip global shortcuts
      if (['INPUT', 'TEXTAREA', 'SELECT'].includes((e.target as HTMLElement).tagName)) {
        if (e.key === 'Escape') {
          (e.target as HTMLElement).blur();
        }
        return;
      }

      if (e.key === ' ' || e.code === 'Space') {
        e.preventDefault();
        this.togglePlayPause();
      } else if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        this.controlPanel.handleExecute();
      } else if ((e.key === 'z' || e.key === 'Z') && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        this.undo();
      } else if (e.key === 'F2') {
        e.preventDefault();
        if (this.activeKeypointIndex >= 0) {
          this.historyDialog.open(this.activeKeypointIndex);
        } else if (this.controller.history.length > 0) {
          this.historyDialog.open(0);
        }
      } else if (e.key === '1') {
        this.renderer.setCameraPreset('iso', true);
      } else if (e.key === '2') {
        this.renderer.setCameraPreset('top', true);
      } else if (e.key === '3') {
        this.renderer.setCameraPreset('front', true);
      } else if (e.key === '4') {
        this.renderer.setCameraPreset('side', true);
      } else if (e.key === 'Escape') {
        this.trajectoryDialog.close();
        this.historyDialog.close();
        this.ioDialog.close();
        this.helpDialog.close();
      }
    });
  }
}
