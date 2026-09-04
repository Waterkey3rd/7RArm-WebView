import './ui/styles/main.css';
import './ui/styles/dialogs.css';
import { App } from './ui/app';

async function bootstrap() {
  const root = document.getElementById('app');
  if (!root) return;

  // Show loading indicator
  root.innerHTML = `
    <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; gap: 16px; background-color: #f8fafc; color: #0f172a; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;">
      <div style="width: 42px; height: 42px; border: 3px solid #e2e8f0; border-top-color: #0284c7; border-radius: 50%; animation: spin 0.8s linear infinite;"></div>
      <div style="font-size: 15px; font-weight: 600;">正在载入 SRS 7R 双臂机器人 WebAssembly 运动学内核...</div>
      <div style="font-size: 12px; color: #64748b;">AdaptiveHybrid IK · Three.js 刚体渲染器</div>
    </div>
    <style>
      @keyframes spin { to { transform: rotate(360deg); } }
    </style>
  `;

  try {
    // In dev & prod, wasm files are served from public /wasm/
    await App.init(root, './wasm/');
  } catch (err: any) {
    root.innerHTML = `
      <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; gap: 14px; background-color: #f8fafc; color: #dc2626; padding: 24px; text-align: center; font-family: sans-serif;">
        <div style="font-size: 38px;">⚠️</div>
        <div style="font-size: 18px; font-weight: 700;">WASM 运动学内核载入失败</div>
        <div style="font-size: 13px; color: #64748b; max-width: 500px;">${err.message || String(err)}</div>
        <button onclick="location.reload()" style="margin-top: 12px; padding: 8px 16px; background: #0284c7; color: white; border: none; border-radius: 6px; cursor: pointer; font-weight: 600;">重新加载</button>
      </div>
    `;
  }
}

bootstrap();
