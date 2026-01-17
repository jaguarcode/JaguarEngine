import React, { useState } from 'react';
import clsx from 'clsx';
import { useWebSocket } from '@/hooks/useWebSocket';
import { useUIStore } from '@/stores/uiStore';
import type { Domain, EntityKind, SpawnEntityRequest } from '@/types';

// Predefined entity templates
const ENTITY_TEMPLATES: Record<Domain, { name: string; kind: EntityKind; template: string }[]> = {
  air: [
    { name: 'F-16 Fighter', kind: 'platform', template: 'aircraft/f16' },
    { name: 'C-130 Transport', kind: 'platform', template: 'aircraft/c130' },
    { name: 'UH-60 Helicopter', kind: 'platform', template: 'aircraft/uh60' },
    { name: 'AIM-120 AMRAAM', kind: 'munition', template: 'missile/aim120' },
    { name: 'Predator UAV', kind: 'platform', template: 'uav/predator' },
  ],
  land: [
    { name: 'M1 Abrams Tank', kind: 'platform', template: 'vehicle/m1abrams' },
    { name: 'HMMWV', kind: 'platform', template: 'vehicle/hmmwv' },
    { name: 'Bradley IFV', kind: 'platform', template: 'vehicle/bradley' },
    { name: 'M777 Howitzer', kind: 'emitter', template: 'artillery/m777' },
    { name: 'Patriot Radar', kind: 'sensor', template: 'radar/patriot' },
  ],
  sea: [
    { name: 'Arleigh Burke DDG', kind: 'platform', template: 'ship/ddg51' },
    { name: 'LCS Frigate', kind: 'platform', template: 'ship/lcs' },
    { name: 'Virginia SSN', kind: 'platform', template: 'submarine/virginia' },
    { name: 'MK48 Torpedo', kind: 'munition', template: 'torpedo/mk48' },
    { name: 'Container Ship', kind: 'platform', template: 'ship/container' },
  ],
  space: [
    { name: 'GPS Satellite', kind: 'platform', template: 'satellite/gps' },
    { name: 'ISS', kind: 'platform', template: 'satellite/iss' },
    { name: 'Comm Satellite', kind: 'emitter', template: 'satellite/comm' },
    { name: 'Debris Object', kind: 'environmental', template: 'debris/small' },
    { name: 'Spy Satellite', kind: 'sensor', template: 'satellite/recon' },
  ],
};

// Default spawn position (Seoul, South Korea)
const DEFAULT_POSITION = {
  latitude: 37.5665,
  longitude: 126.9780,
  altitude: 5000,
};

export const EntitySpawner: React.FC = () => {
  const { spawnEntity } = useWebSocket();
  const cameraPosition = useUIStore((s) => s.view?.cameraPosition);

  const [isCollapsed, setIsCollapsed] = useState(false);
  const [selectedDomain, setSelectedDomain] = useState<Domain>('air');
  const [selectedTemplate, setSelectedTemplate] = useState<string>('');
  const [entityName, setEntityName] = useState<string>('');
  const [position, setPosition] = useState(DEFAULT_POSITION);

  const templates = ENTITY_TEMPLATES[selectedDomain];
  const selectedTemplateData = templates.find((t) => t.template === selectedTemplate);

  const handleSpawn = () => {
    if (!selectedTemplateData) return;

    const request: SpawnEntityRequest = {
      domain: selectedDomain,
      kind: selectedTemplateData.kind,
      name: entityName || selectedTemplateData.name,
      position,
      template: selectedTemplate,
    };

    spawnEntity(request);

    // Reset form
    setEntityName('');
  };

  const handleUseCameraPosition = () => {
    const pos = cameraPosition || DEFAULT_POSITION;
    setPosition({
      latitude: pos.latitude,
      longitude: pos.longitude,
      altitude: selectedDomain === 'space' ? 400000 : selectedDomain === 'air' ? 5000 : 0,
    });
  };

  // Collapsed view - just a button to expand
  if (isCollapsed) {
    return (
      <button
        className="px-4 py-2 bg-gray-800/90 border border-gray-700/50 rounded-lg text-sm text-gray-300 hover:bg-gray-700/90 hover:text-white transition-colors flex items-center gap-2"
        onClick={() => setIsCollapsed(false)}
      >
        <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 4v16m8-8H4" />
        </svg>
        Spawn Entity
      </button>
    );
  }

  return (
    <div className="panel">
      <div className="panel-header">
        <h3 className="panel-title">Spawn Entity</h3>
        <button
          className="text-gray-500 hover:text-gray-300 transition-colors"
          onClick={() => setIsCollapsed(true)}
          title="Hide spawner"
        >
          <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>
      </div>

      <div className="panel-content space-y-4">
        {/* Domain selection */}
        <div>
          <label className="label">Domain</label>
          <div className="flex gap-1">
            {(['air', 'land', 'sea', 'space'] as Domain[]).map((domain) => (
              <button
                key={domain}
                className={clsx(
                  'flex-1 px-2 py-1.5 text-xs rounded capitalize transition-colors',
                  selectedDomain === domain
                    ? clsx({
                        'bg-domain-air text-white': domain === 'air',
                        'bg-domain-land text-white': domain === 'land',
                        'bg-domain-sea text-white': domain === 'sea',
                        'bg-domain-space text-white': domain === 'space',
                      })
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                )}
                onClick={() => {
                  setSelectedDomain(domain);
                  setSelectedTemplate('');
                }}
              >
                {domain}
              </button>
            ))}
          </div>
        </div>

        {/* Template selection */}
        <div>
          <label className="label">Entity Type</label>
          <select
            className="select"
            value={selectedTemplate}
            onChange={(e) => setSelectedTemplate(e.target.value)}
          >
            <option value="">Select an entity...</option>
            {templates.map((t) => (
              <option key={t.template} value={t.template}>
                {t.name} ({t.kind})
              </option>
            ))}
          </select>
        </div>

        {/* Name override */}
        <div>
          <label className="label">Name (optional)</label>
          <input
            type="text"
            className="input"
            placeholder={selectedTemplateData?.name || 'Entity name'}
            value={entityName}
            onChange={(e) => setEntityName(e.target.value)}
          />
        </div>

        {/* Position */}
        <div>
          <div className="flex items-center justify-between mb-1">
            <label className="label mb-0">Position</label>
            <button
              className="text-xs text-primary-400 hover:text-primary-300"
              onClick={handleUseCameraPosition}
            >
              Use camera position
            </button>
          </div>
          <div className="grid grid-cols-3 gap-2">
            <div>
              <label className="text-[10px] text-gray-600">Lat (°)</label>
              <input
                type="number"
                className="input text-xs"
                step="0.001"
                value={position.latitude}
                onChange={(e) =>
                  setPosition({ ...position, latitude: parseFloat(e.target.value) })
                }
              />
            </div>
            <div>
              <label className="text-[10px] text-gray-600">Lon (°)</label>
              <input
                type="number"
                className="input text-xs"
                step="0.001"
                value={position.longitude}
                onChange={(e) =>
                  setPosition({ ...position, longitude: parseFloat(e.target.value) })
                }
              />
            </div>
            <div>
              <label className="text-[10px] text-gray-600">Alt (m)</label>
              <input
                type="number"
                className="input text-xs"
                step="100"
                value={position.altitude}
                onChange={(e) =>
                  setPosition({ ...position, altitude: parseFloat(e.target.value) })
                }
              />
            </div>
          </div>
        </div>

        {/* Spawn button */}
        <button
          className="btn btn-primary w-full"
          onClick={handleSpawn}
          disabled={!selectedTemplate}
        >
          Spawn {selectedTemplateData?.name || 'Entity'}
        </button>
      </div>
    </div>
  );
};

export default EntitySpawner;
