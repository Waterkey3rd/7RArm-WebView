import { defineConfig } from 'vite';
import { teachApi } from './teach-api';

export default defineConfig({
  base: './',
  plugins: [teachApi()],
  worker: { format: 'es' },
});
