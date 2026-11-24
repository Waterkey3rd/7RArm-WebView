import { execFile } from 'node:child_process';
import { existsSync } from 'node:fs';
import { resolve } from 'node:path';
import type { Connect, Plugin } from 'vite';

export function teachApi(): Plugin {
  const executable = process.env.HOSTLINK_TRAJECTORY_BIN ||
    resolve(import.meta.dirname, '../native/build/hostlink_trajectory');
  const handle: Connect.NextHandleFunction = (request, response, next) => {
    if (request.url !== '/api/teach-preview' || request.method !== 'POST') { next(); return; }
    if (request.headers.origin && request.headers.origin !== `http://${request.headers.host}` &&
        request.headers.origin !== `https://${request.headers.host}`) {
      response.writeHead(403).end('仅接受同源预览请求');
      return;
    }
    if (!existsSync(executable)) { response.writeHead(503).end('找不到固件轨迹预览程序'); return; }
    const chunks: Buffer[] = [];
    let size = 0;
    request.on('data', (chunk: Buffer) => {
      size += chunk.length;
      if (size <= 2_000_000) chunks.push(chunk);
    });
    request.on('end', () => {
      if (!size || size > 2_000_000) { response.writeHead(413).end('轨迹输入大小无效'); return; }
      const child = execFile(executable, { timeout: 10_000, maxBuffer: 8_000_000 }, (error, stdout) => {
        if (error) { response.writeHead(422).end('原生轨迹计算失败'); return; }
        response.writeHead(200, { 'Content-Type': 'application/json' }).end(stdout);
      });
      child.stdin?.on('error', () => {});
      child.stdin?.end(Buffer.concat(chunks));
    });
  };
  return {
    name: 'hostlink-teach-preview',
    configureServer(server) { server.middlewares.use(handle); },
    configurePreviewServer(server) { server.middlewares.use(handle); },
  };
}
