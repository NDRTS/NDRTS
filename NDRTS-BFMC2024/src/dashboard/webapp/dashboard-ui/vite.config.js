import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  optimizeDeps: {
    include: [
      '@mantine/core',
      '@mantine/hooks',
      '@tabler/icons-react'
    ],
    // optional: skip deps you never use
    // exclude: ['@codemirror', 'monaco-editor']
  },
  server: {
    port: 3000,
    host: '0.0.0.0',
    hmr: {
      protocol: 'ws',
      host: '0.0.0.0',
      port: 3000,
    },
  }
})
