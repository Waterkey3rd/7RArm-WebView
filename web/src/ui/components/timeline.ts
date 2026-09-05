import type { RoboArmController } from '../../controller';
import type { HistoryPoint } from '../../types';
import { ICONS } from '../icons';

export interface TimelineCallbacks {
  onPlayPause: () => void;
  onStepFrame: (direction: -1 | 1) => void;
  onGoToStart: () => void;
  onGoToEnd: () => void;
  onToggleLoop: (loop: boolean) => void;
  onChangeSpeed: (speed: number) => void;
  onScrub: (progress: number) => void;
  onSelectKeypoint: (index: number) => void;
  onEditKeypoint: (index: number) => void;
  onDeleteKeypoint: (index: number) => void;
}

export class TimelineDock {
  readonly element: HTMLElement;
  private playBtn: HTMLButtonElement;
  private loopBtn: HTMLButtonElement;
  private scrubber: HTMLInputElement;
  private timeLabel: HTMLElement;
  private filmstrip: HTMLElement;

  private isPlaying = false;
  private isLooping = false;
  private selectedIndex = -1;

  constructor(
    private readonly container: HTMLElement,
    private readonly controller: RoboArmController,
    private readonly callbacks: TimelineCallbacks,
  ) {
    this.element = document.createElement('div');
    this.element.className = 'timeline-dock';

    this.element.innerHTML = `
      <!-- Timeline Top Bar: Transport Controls & Scrubber -->
      <div class="timeline-controls-bar">
        <div class="transport-group">
          <button class="btn btn-secondary btn-icon btn-sm" id="btn-tl-start" title="跳至序列开始 (Home)">${ICONS.START}</button>
          <button class="btn btn-secondary btn-icon btn-sm" id="btn-tl-prev" title="上一关键点 (Left Arrow)">${ICONS.PREV}</button>
          <button class="btn btn-primary btn-sm" id="btn-tl-play" title="播放/暂停 (Space)" style="min-width: 80px;">
            <span id="play-icon" class="btn-icon-slot">${ICONS.PLAY}</span>
            <span id="play-text">播放</span>
          </button>
          <button class="btn btn-secondary btn-icon btn-sm" id="btn-tl-next" title="下一关键点 (Right Arrow)">${ICONS.NEXT}</button>
          <button class="btn btn-secondary btn-icon btn-sm" id="btn-tl-end" title="跳至序列末尾 (End)">${ICONS.END}</button>
          <button class="btn btn-ghost btn-sm" id="btn-tl-loop" title="切换循环播放">
            <span class="btn-icon-slot">${ICONS.LOOP}</span>
            <span id="loop-text">单次</span>
          </button>

          <select class="form-select" id="select-tl-speed" style="padding: 2px 6px; font-size: 11px; height: 26px;">
            <option value="0.5">0.5x</option>
            <option value="1.0" selected>1.0x</option>
            <option value="2.0">2.0x</option>
            <option value="4.0">4.0x</option>
          </select>
        </div>

        <div class="timeline-scrubber-track">
          <input type="range" class="timeline-slider" id="timeline-scrubber" min="0" max="1000" value="0">
        </div>

        <div style="font-family: var(--font-mono); font-size: 12px; font-weight: 600; color: var(--text-secondary);" id="timeline-time-display">
          0.00s / 0.00s (0 关键点)
        </div>
      </div>

      <!-- Keypoints Cards Filmstrip -->
      <div class="keypoints-filmstrip" id="keypoints-filmstrip">
        <!-- Rendered dynamically -->
      </div>
    `;

    container.appendChild(this.element);

    this.playBtn = this.element.querySelector('#btn-tl-play') as HTMLButtonElement;
    this.loopBtn = this.element.querySelector('#btn-tl-loop') as HTMLButtonElement;
    this.scrubber = this.element.querySelector('#timeline-scrubber') as HTMLInputElement;
    this.timeLabel = this.element.querySelector('#timeline-time-display') as HTMLElement;
    this.filmstrip = this.element.querySelector('#keypoints-filmstrip') as HTMLElement;

    // Button event listeners
    this.playBtn.addEventListener('click', () => this.callbacks.onPlayPause());
    this.element.querySelector('#btn-tl-start')?.addEventListener('click', () => this.callbacks.onGoToStart());
    this.element.querySelector('#btn-tl-prev')?.addEventListener('click', () => this.callbacks.onStepFrame(-1));
    this.element.querySelector('#btn-tl-next')?.addEventListener('click', () => this.callbacks.onStepFrame(1));
    this.element.querySelector('#btn-tl-end')?.addEventListener('click', () => this.callbacks.onGoToEnd());

    this.loopBtn.addEventListener('click', () => {
      this.isLooping = !this.isLooping;
      this.loopBtn.innerHTML = `<span class="btn-icon-slot">${ICONS.LOOP}</span><span id="loop-text">${this.isLooping ? '循环' : '单次'}</span>`;
      this.loopBtn.style.color = this.isLooping ? 'var(--text-primary)' : 'inherit';
      this.callbacks.onToggleLoop(this.isLooping);
    });

    const speedSelect = this.element.querySelector('#select-tl-speed') as HTMLSelectElement;
    speedSelect.addEventListener('change', () => {
      this.callbacks.onChangeSpeed(parseFloat(speedSelect.value) || 1.0);
    });

    this.scrubber.addEventListener('input', () => {
      const progress = (parseFloat(this.scrubber.value) || 0) / 1000;
      this.callbacks.onScrub(progress);
    });

    this.updateTimeline();
  }

  setPlayState(playing: boolean): void {
    this.isPlaying = playing;
    const icon = this.element.querySelector('#play-icon');
    const text = this.element.querySelector('#play-text');
    if (icon && text) {
      icon.innerHTML = playing ? ICONS.PAUSE : ICONS.PLAY;
      text.textContent = playing ? '暂停' : '播放';
    }
  }

  setProgress(progress: number, currentTimeS: number, totalTimeS: number): void {
    if (!this.scrubber.matches(':active')) {
      this.scrubber.value = String(Math.round(progress * 1000));
    }
    const count = this.controller.history.length;
    this.timeLabel.textContent = `${currentTimeS.toFixed(2)}s / ${totalTimeS.toFixed(2)}s (${count} 关键点)`;
  }

  updateTimeline(activePointIndex = -1): void {
    const history = this.controller.history;
    this.filmstrip.innerHTML = '';
    this.selectedIndex = activePointIndex;

    const plannedDurations = history.map(point => this.controller.plannedDuration(point));
    let totalDurationMs = 0;
    plannedDurations.forEach(duration => totalDurationMs += duration);
    const totalTimeS = totalDurationMs / 1000;
    this.timeLabel.textContent = `0.00s / ${totalTimeS.toFixed(2)}s (${history.length} 关键点)`;

    if (history.length === 0) {
      this.filmstrip.innerHTML = `
        <div style="color: var(--text-muted); font-size: 13px; margin: auto; display: flex; align-items: center; gap: 8px;">
          <span style="display: inline-flex;">${ICONS.INFO}</span>
          <span>尚无动作关键点。可通过右侧控制台执行运动，或点击上方“函数轨迹”添加关键点</span>
        </div>
      `;
      return;
    }

    history.forEach((point, idx) => {
      const card = document.createElement('div');
      card.className = `keypoint-card ${idx === activePointIndex ? 'active' : ''}`;

      const isCartesian = point.frameTargets.left.space === 'CartesianSpace' || point.frameTargets.right.space === 'CartesianSpace';
      const spaceLabel = isCartesian ? '笛卡尔' : '关节角';
      const spaceClass = isCartesian ? 'cartesian' : 'joint';

      card.innerHTML = `
        <div class="keypoint-card-top">
          <span class="keypoint-index">#${idx + 1}</span>
          <span class="space-badge ${spaceClass}">${spaceLabel}</span>
        </div>
        <div class="keypoint-label" title="${escapeHtml(point.label)}">${escapeHtml(point.label)}</div>
        <div class="keypoint-card-bottom">
          <span style="display: flex; align-items: center; gap: 4px;" title="请求时长 / 实机规划后时长"><span class="btn-icon-slot">${ICONS.CLOCK}</span><span>${point.durationMs}ms${plannedDurations[idx] > point.durationMs + 1 ? ` → ${Math.round(plannedDurations[idx])}ms` : ''}</span></span>
          <div style="display: flex; gap: 4px;">
            <button class="btn btn-ghost btn-sm" style="padding: 2px 4px; font-size: 11px;" id="kp-edit-${idx}" title="编辑关键点 (F2)">${ICONS.EDIT}</button>
            <button class="btn btn-ghost btn-sm" style="padding: 2px 4px; font-size: 11px; color: var(--status-danger);" id="kp-del-${idx}" title="删除关键点">${ICONS.TRASH}</button>
          </div>
        </div>
      `;

      // Click to select/restore
      card.addEventListener('click', (e) => {
        if ((e.target as HTMLElement).closest('button')) return;
        this.callbacks.onSelectKeypoint(idx);
      });

      card.querySelector(`#kp-edit-${idx}`)?.addEventListener('click', (e) => {
        e.stopPropagation();
        this.callbacks.onEditKeypoint(idx);
      });

      card.querySelector(`#kp-del-${idx}`)?.addEventListener('click', (e) => {
        e.stopPropagation();
        this.callbacks.onDeleteKeypoint(idx);
      });

      this.filmstrip.appendChild(card);
    });

    if (activePointIndex >= 0) {
      const activeCard = this.filmstrip.children[activePointIndex] as HTMLElement;
      if (activeCard) {
        activeCard.scrollIntoView({ behavior: 'smooth', inline: 'nearest', block: 'nearest' });
      }
    }
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
