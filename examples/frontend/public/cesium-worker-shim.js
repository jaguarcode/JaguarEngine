// AMD shim for Cesium workers
// This prevents "define is not defined" errors from Lerc's UMD check
if (typeof define === 'undefined') {
  self.define = function(deps, factory) {
    if (typeof deps === 'function') {
      factory = deps;
      deps = [];
    }
    const result = factory();
    if (typeof module !== 'undefined' && module.exports) {
      module.exports = result;
    }
    return result;
  };
  self.define.amd = false; // Don't trigger AMD mode
}
