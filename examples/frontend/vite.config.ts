import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import cesium from 'vite-plugin-cesium';
import { viteStaticCopy } from 'vite-plugin-static-copy';
import path from 'path';

// Get Cesium path for static copy
const cesiumSource = 'node_modules/cesium/Build/CesiumUnminified';

export default defineConfig({
  plugins: [
    react(),
    cesium({
      // Use the pre-built Cesium (not rebuilding from source)
      rebuildCesium: false,
      // Use unminified build in development for better debugging
      devMinifyCesium: false,
    }),
    // Copy Cesium static assets for build
    viteStaticCopy({
      targets: [
        { src: `${cesiumSource}/Workers/*`, dest: 'cesium/Workers' },
        { src: `${cesiumSource}/ThirdParty/*`, dest: 'cesium/ThirdParty' },
        { src: `${cesiumSource}/Assets/*`, dest: 'cesium/Assets' },
        { src: `${cesiumSource}/Widgets/*`, dest: 'cesium/Widgets' },
      ],
    }),
  ],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  define: {
    // Ensure CESIUM_BASE_URL is defined globally
    CESIUM_BASE_URL: JSON.stringify('/cesium/'),
  },
  server: {
    port: 3000,
    proxy: {
      '/api': {
        target: 'http://localhost:8080',
        changeOrigin: true,
      },
      '/ws': {
        target: 'ws://localhost:8081',
        ws: true,
      },
    },
  },
  optimizeDeps: {
    // Include CommonJS dependencies to fix ESM compatibility issues with Cesium
    include: [
      'mersenne-twister',
      'urijs',
      'grapheme-splitter',
      'autolinker',
      'bitmap-sdf',
      'dompurify',
      'earcut',
      'jsep',
      'kdbush',
      'lerc',
      'pako',
      'rbush',
      'topojson-client',
      'nosleep.js',
    ],
    // Exclude Cesium from pre-bundling - let vite-plugin-cesium handle it
    exclude: ['cesium'],
  },
  build: {
    rollupOptions: {
      output: {
        manualChunks: {
          react: ['react', 'react-dom'],
          zustand: ['zustand'],
          charts: ['recharts'],
        },
      },
    },
  },
});
