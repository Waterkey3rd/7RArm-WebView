import type { RoboArmController } from '../../controller';
import { ICONS } from '../icons';
import { showToast } from './toast';

export class IoDialog {
  private backdrop: HTMLElement | null = null;
  private currentTab: 'export' | 'import' = 'export';
  private forceJointSpace = false;
  private exportFrom = 0;
  private exportTo = -1;

  constructor(
    private readonly controller: RoboArmController,
    private readonly onSequenceChanged: () => void,
  ) {}

  open(initialTab: 'export' | 'import' = 'export'): void {
    if (this.backdrop) this.close();

    this.currentTab = initialTab;
    this.exportFrom = 0;
    this.exportTo = this.controller.history.length - 1;
    this.backdrop = document.createElement('div');
    this.backdrop.className = 'modal-backdrop';

    this.backdrop.innerHTML = `
      <div class="modal-window" style="max-width: 740px;">
        <div class="modal-header">
          <div class="modal-title">
            <span class="btn-icon-slot">${ICONS.IO}</span>
            <span>动作序列管理 (performance-action-sequence-v2)</span>
          </div>
          <button class="modal-close-btn" id="modal-close-btn">${ICONS.CLOSE}</button>
        </div>

        <div style="display: flex; border-bottom: 1px solid var(--border-color); background: var(--bg-surface-subtle); padding: 0 16px;">
          <button class="btn btn-ghost btn-sm ${this.currentTab === 'export' ? 'active' : ''}" id="tab-btn-export" style="border-radius: 0; border-bottom: 2px solid ${this.currentTab === 'export' ? 'var(--left-arm-primary)' : 'transparent'}; font-weight: 700;">
            导出序列 JSON
          </button>
          <button class="btn btn-ghost btn-sm ${this.currentTab === 'import' ? 'active' : ''}" id="tab-btn-import" style="border-radius: 0; border-bottom: 2px solid ${this.currentTab === 'import' ? 'var(--left-arm-primary)' : 'transparent'}; font-weight: 700;">
            导入序列 JSON
          </button>
        </div>

        <div class="modal-body" id="io-modal-content"></div>

        <div class="modal-footer" id="io-modal-footer"></div>
      </div>
    `;

    document.body.appendChild(this.backdrop);

    this.backdrop.querySelector('#modal-close-btn')?.addEventListener('click', () => this.close());
    this.backdrop.querySelector('#tab-btn-export')?.addEventListener('click', () => {
      this.currentTab = 'export';
      this.updateTabsHeader();
      this.renderTabContent();
    });
    this.backdrop.querySelector('#tab-btn-import')?.addEventListener('click', () => {
      this.currentTab = 'import';
      this.updateTabsHeader();
      this.renderTabContent();
    });

    this.renderTabContent();
  }

  private updateTabsHeader(): void {
    const expBtn = this.backdrop?.querySelector('#tab-btn-export') as HTMLElement;
    const impBtn = this.backdrop?.querySelector('#tab-btn-import') as HTMLElement;
    if (expBtn && impBtn) {
      expBtn.style.borderBottomColor = this.currentTab === 'export' ? 'var(--left-arm-primary)' : 'transparent';
      impBtn.style.borderBottomColor = this.currentTab === 'import' ? 'var(--left-arm-primary)' : 'transparent';
    }
  }

  private renderTabContent(): void {
    const body = this.backdrop?.querySelector('#io-modal-content') as HTMLElement;
    const footer = this.backdrop?.querySelector('#io-modal-footer') as HTMLElement;
    if (!body || !footer) return;

    if (this.currentTab === 'export') {
      this.renderExportTab(body, footer);
    } else {
      this.renderImportTab(body, footer);
    }
  }

  private renderExportTab(body: HTMLElement, footer: HTMLElement): void {
    let jsonStr = '';
    try {
      const data = this.controller.export(this.forceJointSpace, this.exportFrom, this.exportTo);
      jsonStr = JSON.stringify(data, null, 2);
    } catch (err: any) {
      jsonStr = `// 无法导出: ${err.message || '没有历史关键点'}`;
    }

    body.innerHTML = `
      <div style="display: flex; align-items: center; justify-content: space-between;">
        <div style="font-size: 13px; font-weight: 700; color: var(--text-secondary);">导出目标空间规范:</div>
        <select class="form-select" id="select-export-space" style="width: 260px;">
          <option value="mixed" ${!this.forceJointSpace ? 'selected' : ''}>保留记录空间 (混合 Joint / Cartesian)</option>
          <option value="joint" ${this.forceJointSpace ? 'selected' : ''}>全部强制转为 JointAngleSpace</option>
        </select>
      </div>

      <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 12px;">
        <div class="form-field">
          <label class="form-field-label">起始关键点（包含）</label>
          <select class="form-select" id="select-export-from">${this.rangeOptions(this.exportFrom)}</select>
        </div>
        <div class="form-field">
          <label class="form-field-label">结束关键点（包含）</label>
          <select class="form-select" id="select-export-to">${this.rangeOptions(this.exportTo)}</select>
        </div>
      </div>

      <div class="alert-banner info" style="margin-top: 10px;">
        <span style="display: inline-flex; margin-top: 1px;">${ICONS.INFO}</span>
        <span>将导出第 ${this.exportFrom + 1}～${this.exportTo + 1} 个关键点。文件第 0 帧会自动写入所选片段开始前的关节状态，因此中间片段可以独立播放。</span>
      </div>

      <div class="json-preview-box" id="json-preview">${escapeHtml(jsonStr)}</div>
    `;

    const sel = body.querySelector('#select-export-space') as HTMLSelectElement;
    sel.addEventListener('change', () => {
      this.forceJointSpace = sel.value === 'joint';
      this.renderTabContent();
    });
    const fromSelect = body.querySelector('#select-export-from') as HTMLSelectElement;
    const toSelect = body.querySelector('#select-export-to') as HTMLSelectElement;
    fromSelect.addEventListener('change', () => {
      this.exportFrom = Number(fromSelect.value);
      if (this.exportTo < this.exportFrom) this.exportTo = this.exportFrom;
      this.renderTabContent();
    });
    toSelect.addEventListener('change', () => {
      this.exportTo = Number(toSelect.value);
      if (this.exportFrom > this.exportTo) this.exportFrom = this.exportTo;
      this.renderTabContent();
    });

    footer.innerHTML = `
      <button class="btn btn-secondary" id="btn-copy-json">
        <span class="btn-icon-slot">${ICONS.COPY}</span>
        <span>复制到剪贴板</span>
      </button>
      <button class="btn btn-primary" id="btn-download-json">
        <span class="btn-icon-slot">${ICONS.DOWNLOAD}</span>
        <span>下载 JSON 文件</span>
      </button>
    `;

    footer.querySelector('#btn-copy-json')?.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(jsonStr);
        showToast({ title: '复制成功', message: '已将动作序列 JSON 复制到剪贴板', type: 'success' });
      } catch {
        showToast({ message: '剪贴板访问失败，请手动复制', type: 'warning' });
      }
    });

    footer.querySelector('#btn-download-json')?.addEventListener('click', () => {
      const blob = new Blob([jsonStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `actionsequence_${this.exportFrom + 1}-${this.exportTo + 1}_${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
      showToast({ title: '已下载', message: '动作序列文件已保存', type: 'success' });
    });
  }

  private rangeOptions(selected: number): string {
    return this.controller.history.map((point, index) =>
      `<option value="${index}" ${index === selected ? 'selected' : ''}>#${index + 1} ${escapeHtml(point.label)}</option>`
    ).join('');
  }

  private renderImportTab(body: HTMLElement, footer: HTMLElement): void {
    body.innerHTML = `
      <div class="file-dropzone" id="file-dropzone">
        <div class="dropzone-icon" style="color: var(--text-secondary);">${ICONS.UPLOAD}</div>
        <div class="dropzone-title">拖拽 JSON 文件到此区域，或点击选择文件</div>
        <div class="dropzone-desc">支持 performance-action-sequence-v2 规范动作序列文件</div>
        <input type="file" id="file-input" accept=".json" style="display: none;">
      </div>

      <div style="display: flex; flex-direction: column; gap: 6px;">
        <label class="form-field-label">或直接在此粘贴 JSON 文本：</label>
        <textarea class="form-input" id="paste-json-area" rows="6" placeholder="粘贴完整的 performance-action-sequence-v2 JSON 内容..." style="font-family: var(--font-mono); font-size: 11px;"></textarea>
      </div>
    `;

    const dropzone = body.querySelector('#file-dropzone') as HTMLElement;
    const fileInput = body.querySelector('#file-input') as HTMLInputElement;
    const textArea = body.querySelector('#paste-json-area') as HTMLTextAreaElement;

    dropzone.addEventListener('click', () => fileInput.click());
    dropzone.addEventListener('dragover', (e) => {
      e.preventDefault();
      dropzone.classList.add('dragover');
    });
    dropzone.addEventListener('dragleave', () => dropzone.classList.remove('dragover'));
    dropzone.addEventListener('drop', (e) => {
      e.preventDefault();
      dropzone.classList.remove('dragover');
      if (e.dataTransfer?.files.length) {
        this.readFile(e.dataTransfer.files[0]);
      }
    });

    fileInput.addEventListener('change', () => {
      if (fileInput.files?.length) {
        this.readFile(fileInput.files[0]);
      }
    });

    footer.innerHTML = `
      <button class="btn btn-secondary" id="btn-cancel-import">取消</button>
      <button class="btn btn-primary" id="btn-parse-json">解析并载入序列</button>
    `;

    footer.querySelector('#btn-cancel-import')?.addEventListener('click', () => this.close());
    footer.querySelector('#btn-parse-json')?.addEventListener('click', () => {
      const text = textArea.value.trim();
      if (!text) {
        showToast({ title: '输入为空', message: '请先选择文件或粘贴 JSON 文本', type: 'warning' });
        return;
      }
      this.parseAndLoad(text);
    });
  }

  private readFile(file: File): void {
    const reader = new FileReader();
    reader.onload = (e) => {
      const content = e.target?.result as string;
      this.parseAndLoad(content);
    };
    reader.readAsText(file);
  }

  private parseAndLoad(jsonText: string): void {
    try {
      const payload = JSON.parse(jsonText);
      this.controller.import(payload);
      showToast({
        title: '导入成功',
        message: `成功导入动作序列，包含 ${this.controller.history.length} 个关键点，已完成逆运动学校验`,
        type: 'success',
      });
      this.close();
      this.onSequenceChanged();
    } catch (err: any) {
      showToast({
        title: '导入解析失败',
        message: err.message || 'JSON 格式不符合 performance-action-sequence-v2 规范或姿态不可达',
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
