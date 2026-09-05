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
    if (typeof window !== 'undefined' && window.location.protocol === 'file:') {
      throw new Error('检测到以 file:// 协议直接打开。受浏览器安全跨域策略限制，WebAssembly 必须在 HTTP 服务下运行。请在终端执行：npm run dev 或 npx vite preview，然后访问提示的本地网址 (如 http://localhost:5173)。');
    }
    const wasmBase = typeof window !== 'undefined'
      ? new URL('wasm/', window.location.href).href
      : '/wasm/';
    await App.init(root, wasmBase);
  } catch (err: any) {
    root.innerHTML = `
      <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; gap: 14px; background-color: #f8fafc; color: #0f172a; padding: 24px; text-align: center; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;">
        <div style="width: 48px; height: 48px; border-radius: 50%; background: #fee2e2; color: #dc2626; display: flex; align-items: center; justify-content: center; font-size: 24px; font-weight: bold;">!</div>
        <div style="font-size: 18px; font-weight: 700; color: #0f172a;">WASM 运动学内核载入失败</div>
        <div style="font-size: 13px; color: #64748b; max-width: 540px; line-height: 1.6;">${err.message || String(err)}</div>
        <button onclick="location.reload()" style="margin-top: 14px; padding: 8px 20px; background: #0284c7; color: white; border: none; border-radius: 6px; cursor: pointer; font-weight: 600; font-size: 13px;">重新加载</button>
      </div>
    `;
  }
}

bootstrap();
