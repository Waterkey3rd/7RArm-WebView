import { ICONS } from '../icons';

export type ToastType = 'info' | 'success' | 'warning' | 'danger';

export interface ToastOptions {
  title?: string;
  message: string;
  type?: ToastType;
  duration?: number;
}

let container: HTMLElement | null = null;

function getOrCreateContainer(): HTMLElement {
  if (!container || !document.body.contains(container)) {
    container = document.createElement('div');
    container.className = 'toast-container';
    document.body.appendChild(container);
  }
  return container;
}

const TOAST_ICONS: Record<ToastType, string> = {
  info: ICONS.INFO,
  success: ICONS.CHECK,
  warning: ICONS.WARN,
  danger: ICONS.CROSS,
};

const DEFAULT_TITLES: Record<ToastType, string> = {
  info: '提示',
  success: '成功',
  warning: '警告',
  danger: '错误',
};

export function showToast(options: ToastOptions | string): void {
  const opts: ToastOptions = typeof options === 'string' ? { message: options } : options;
  const type: ToastType = opts.type || 'info';
  const title = opts.title || DEFAULT_TITLES[type];
  const duration = opts.duration ?? (type === 'danger' ? 5000 : 3200);

  const parent = getOrCreateContainer();
  const item = document.createElement('div');
  item.className = `toast-item toast-${type}`;

  item.innerHTML = `
    <span class="toast-icon" style="display: flex; align-items: center;">${TOAST_ICONS[type]}</span>
    <div class="toast-content">
      <div class="toast-title">${escapeHtml(title)}</div>
      <div class="toast-message">${escapeHtml(opts.message)}</div>
    </div>
  `;

  parent.appendChild(item);

  const dismiss = () => {
    item.style.opacity = '0';
    item.style.transform = 'translateX(30px) scale(0.95)';
    setTimeout(() => {
      if (item.parentElement) item.parentElement.removeChild(item);
    }, 200);
  };

  item.addEventListener('click', dismiss);
  if (duration > 0) {
    setTimeout(dismiss, duration);
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
