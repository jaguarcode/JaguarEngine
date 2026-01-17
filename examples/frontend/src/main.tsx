import ReactDOM from 'react-dom/client';
import App from './App';
import './index.css';

// Cesium configuration
import { Ion } from 'cesium';
import 'cesium/Build/Cesium/Widgets/widgets.css';

// Set Cesium Ion default access token from environment variable
if (import.meta.env.VITE_CESIUM_TOKEN) {
  Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;
}

// Note: StrictMode is disabled because Cesium viewer doesn't work well with it
// due to double-mounting causing WebGL context issues
ReactDOM.createRoot(document.getElementById('root')!).render(<App />);
