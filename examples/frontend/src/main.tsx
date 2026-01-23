// IMPORTANT: Cesium setup must happen before any Cesium imports
// This must be done synchronously before the Cesium module loads
declare global {
  interface Window {
    CESIUM_BASE_URL: string;
    define?: unknown;
  }
  const CESIUM_BASE_URL: string;
}

// Set window.CESIUM_BASE_URL from the Vite-defined constant
// This is needed because workers check window.CESIUM_BASE_URL
if (typeof CESIUM_BASE_URL !== 'undefined') {
  window.CESIUM_BASE_URL = CESIUM_BASE_URL;
}

// Provide a minimal AMD define shim if not available
// This prevents "define is not defined" errors from Cesium's bundled Lerc library
// which uses UMD pattern checking for AMD define before CommonJS exports
if (typeof window.define === 'undefined') {
  // Create a no-op define that won't trigger AMD detection
  // The UMD check is: typeof define=="function" && define.amd
  // By not setting define.amd, we make the library fall through to CommonJS/global
  (window as Window).define = undefined;
}

import ReactDOM from 'react-dom/client';
import App from './App';
import './index.css';

// Now import Cesium after setup is complete
import * as Cesium from 'cesium';
// Import Cesium widget styles
import 'cesium/Source/Widgets/widgets.css';

// Set Cesium Ion default access token from environment variable
if (import.meta.env.VITE_CESIUM_TOKEN) {
  Cesium.Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;
}

// Note: StrictMode is disabled because Cesium viewer doesn't work well with it
// due to double-mounting causing WebGL context issues
ReactDOM.createRoot(document.getElementById('root')!).render(<App />);
