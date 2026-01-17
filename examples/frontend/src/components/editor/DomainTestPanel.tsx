import React, { useState } from 'react';
import clsx from 'clsx';
import { useWebSocket } from '@/hooks/useWebSocket';
import { useEntityStore, selectSelectedEntity } from '@/stores/entityStore';
import type { Domain, AircraftState, GroundVehicleState, ShipState } from '@/types';

// Slider component
const ControlSlider: React.FC<{
  label: string;
  value: number;
  min?: number;
  max?: number;
  step?: number;
  unit?: string;
  onChange: (value: number) => void;
}> = ({ label, value, min = -1, max = 1, step = 0.01, unit, onChange }) => (
  <div className="space-y-1">
    <div className="flex items-center justify-between">
      <label className="text-xs text-gray-400">{label}</label>
      <span className="text-xs font-mono text-gray-300">
        {value.toFixed(2)}
        {unit && <span className="text-gray-500 ml-1">{unit}</span>}
      </span>
    </div>
    <input
      type="range"
      className="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer"
      min={min}
      max={max}
      step={step}
      value={value}
      onChange={(e) => onChange(parseFloat(e.target.value))}
    />
  </div>
);

// Air domain controls
const AirDomainControls: React.FC = () => {
  const entity = useEntityStore(selectSelectedEntity) as AircraftState | null;
  const { setFlightControls, setAutopilot } = useWebSocket();

  const [controls, setControls] = useState({
    elevator: 0,
    aileron: 0,
    rudder: 0,
    throttle: 0.5,
  });

  if (!entity || entity.domain !== 'air' || entity.kind !== 'platform') {
    return (
      <div className="text-center text-gray-500 py-4">
        Select an aircraft to control
      </div>
    );
  }

  const handleControlChange = (key: keyof typeof controls, value: number) => {
    const newControls = { ...controls, [key]: value };
    setControls(newControls);
    setFlightControls(entity.id, newControls);
  };

  return (
    <div className="space-y-4">
      {/* Flight controls */}
      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Flight Controls
        </h4>
        <div className="space-y-3">
          <ControlSlider
            label="Elevator (Pitch)"
            value={controls.elevator}
            onChange={(v) => handleControlChange('elevator', v)}
          />
          <ControlSlider
            label="Aileron (Roll)"
            value={controls.aileron}
            onChange={(v) => handleControlChange('aileron', v)}
          />
          <ControlSlider
            label="Rudder (Yaw)"
            value={controls.rudder}
            onChange={(v) => handleControlChange('rudder', v)}
          />
          <ControlSlider
            label="Throttle"
            value={controls.throttle}
            min={0}
            max={1}
            unit="%"
            onChange={(v) => handleControlChange('throttle', v)}
          />
        </div>
      </div>

      {/* Autopilot modes */}
      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Autopilot
        </h4>
        <div className="grid grid-cols-2 gap-2">
          <button
            className="btn btn-secondary text-xs"
            onClick={() => setAutopilot(entity.id, 'altitude_hold', { altitude: entity.altitude })}
          >
            Alt Hold
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => setAutopilot(entity.id, 'heading_hold', { heading: entity.heading })}
          >
            Hdg Hold
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => setAutopilot(entity.id, 'speed_hold', { speed: entity.airspeed })}
          >
            Spd Hold
          </button>
          <button
            className="btn btn-danger text-xs"
            onClick={() => setAutopilot(entity.id, 'off')}
          >
            A/P Off
          </button>
        </div>
      </div>

      {/* Quick actions */}
      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Actions
        </h4>
        <div className="grid grid-cols-2 gap-2">
          <button
            className="btn btn-primary text-xs"
            onClick={() => {
              handleControlChange('throttle', 1.0);
            }}
          >
            Full Throttle
          </button>
          <button
            className="btn btn-warning text-xs"
            onClick={() => {
              handleControlChange('throttle', 0);
            }}
          >
            Idle
          </button>
        </div>
      </div>
    </div>
  );
};

// Land domain controls
const LandDomainControls: React.FC = () => {
  const entity = useEntityStore(selectSelectedEntity) as GroundVehicleState | null;
  const { setVehicleControls } = useWebSocket();

  const [controls, setControls] = useState({
    throttle: 0,
    steering: 0,
    brake: 0,
  });

  if (!entity || entity.domain !== 'land' || entity.kind !== 'platform') {
    return (
      <div className="text-center text-gray-500 py-4">
        Select a ground vehicle to control
      </div>
    );
  }

  const handleControlChange = (key: keyof typeof controls, value: number) => {
    const newControls = { ...controls, [key]: value };
    setControls(newControls);
    setVehicleControls(entity.id, newControls);
  };

  return (
    <div className="space-y-4">
      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Vehicle Controls
        </h4>
        <div className="space-y-3">
          <ControlSlider
            label="Throttle"
            value={controls.throttle}
            onChange={(v) => handleControlChange('throttle', v)}
          />
          <ControlSlider
            label="Steering"
            value={controls.steering}
            onChange={(v) => handleControlChange('steering', v)}
          />
          <ControlSlider
            label="Brake"
            value={controls.brake}
            min={0}
            max={1}
            onChange={(v) => handleControlChange('brake', v)}
          />
        </div>
      </div>

      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Quick Actions
        </h4>
        <div className="grid grid-cols-2 gap-2">
          <button
            className="btn btn-primary text-xs"
            onClick={() => handleControlChange('throttle', 1)}
          >
            Forward
          </button>
          <button
            className="btn btn-warning text-xs"
            onClick={() => handleControlChange('throttle', -1)}
          >
            Reverse
          </button>
          <button
            className="btn btn-danger text-xs"
            onClick={() => handleControlChange('brake', 1)}
          >
            Full Brake
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => {
              handleControlChange('throttle', 0);
              handleControlChange('steering', 0);
              handleControlChange('brake', 0);
            }}
          >
            Neutral
          </button>
        </div>
      </div>
    </div>
  );
};

// Sea domain controls
const SeaDomainControls: React.FC = () => {
  const entity = useEntityStore(selectSelectedEntity) as ShipState | null;
  const { setShipControls } = useWebSocket();

  const [controls, setControls] = useState({
    rudder: 0,
    throttle: 0.5,
  });

  if (!entity || entity.domain !== 'sea' || entity.kind !== 'platform') {
    return (
      <div className="text-center text-gray-500 py-4">
        Select a ship to control
      </div>
    );
  }

  const handleControlChange = (key: keyof typeof controls, value: number) => {
    const newControls = { ...controls, [key]: value };
    setControls(newControls);
    setShipControls(entity.id, newControls);
  };

  return (
    <div className="space-y-4">
      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Ship Controls
        </h4>
        <div className="space-y-3">
          <ControlSlider
            label="Rudder"
            value={controls.rudder}
            onChange={(v) => handleControlChange('rudder', v)}
          />
          <ControlSlider
            label="Throttle"
            value={controls.throttle}
            min={0}
            max={1}
            onChange={(v) => handleControlChange('throttle', v)}
          />
        </div>
      </div>

      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Helm Orders
        </h4>
        <div className="grid grid-cols-3 gap-2">
          <button
            className="btn btn-secondary text-xs"
            onClick={() => handleControlChange('rudder', -1)}
          >
            Hard Port
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => handleControlChange('rudder', 0)}
          >
            Midships
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => handleControlChange('rudder', 1)}
          >
            Hard Stbd
          </button>
          <button
            className="btn btn-primary text-xs"
            onClick={() => handleControlChange('throttle', 1)}
          >
            Full Ahead
          </button>
          <button
            className="btn btn-warning text-xs"
            onClick={() => handleControlChange('throttle', 0)}
          >
            All Stop
          </button>
          <button
            className="btn btn-danger text-xs"
            onClick={() => handleControlChange('throttle', -0.5)}
          >
            Full Astern
          </button>
        </div>
      </div>
    </div>
  );
};

// Space domain controls (view only - orbits are computed)
const SpaceDomainControls: React.FC = () => {
  const entity = useEntityStore(selectSelectedEntity);
  const { sendCommand } = useWebSocket();

  if (!entity || entity.domain !== 'space') {
    return (
      <div className="text-center text-gray-500 py-4">
        Select a spacecraft to view
      </div>
    );
  }

  return (
    <div className="space-y-4">
      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Orbital Maneuvers
        </h4>
        <div className="grid grid-cols-2 gap-2">
          <button
            className="btn btn-primary text-xs"
            onClick={() => sendCommand({ command: 'prograde_burn', entityId: entity.id, params: { deltaV: 10 } })}
          >
            Prograde +10 m/s
          </button>
          <button
            className="btn btn-warning text-xs"
            onClick={() => sendCommand({ command: 'retrograde_burn', entityId: entity.id, params: { deltaV: 10 } })}
          >
            Retrograde -10 m/s
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => sendCommand({ command: 'normal_burn', entityId: entity.id, params: { deltaV: 10 } })}
          >
            Normal +10 m/s
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => sendCommand({ command: 'antinormal_burn', entityId: entity.id, params: { deltaV: 10 } })}
          >
            Anti-Normal -10 m/s
          </button>
        </div>
      </div>

      <div>
        <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
          Orbit Propagation
        </h4>
        <div className="grid grid-cols-2 gap-2">
          <button
            className="btn btn-secondary text-xs"
            onClick={() => sendCommand({ command: 'show_orbit', entityId: entity.id })}
          >
            Show Orbit
          </button>
          <button
            className="btn btn-secondary text-xs"
            onClick={() => sendCommand({ command: 'show_ground_track', entityId: entity.id })}
          >
            Ground Track
          </button>
        </div>
      </div>
    </div>
  );
};

// Main domain test panel
export const DomainTestPanel: React.FC = () => {
  const [activeDomain, setActiveDomain] = useState<Domain>('air');

  return (
    <div className="panel h-full flex flex-col">
      <div className="panel-header">
        <h3 className="panel-title">Domain Controls</h3>
      </div>

      {/* Domain tabs */}
      <div className="flex border-b border-gray-700/50">
        {(['air', 'land', 'sea', 'space'] as Domain[]).map((domain) => (
          <button
            key={domain}
            className={clsx(
              'flex-1 px-3 py-2 text-xs font-medium transition-colors border-b-2',
              activeDomain === domain
                ? clsx('border-current', {
                    'text-domain-air': domain === 'air',
                    'text-domain-land': domain === 'land',
                    'text-domain-sea': domain === 'sea',
                    'text-domain-space': domain === 'space',
                  })
                : 'border-transparent text-gray-500 hover:text-gray-400'
            )}
            onClick={() => setActiveDomain(domain)}
          >
            {domain.toUpperCase()}
          </button>
        ))}
      </div>

      {/* Domain-specific controls */}
      <div className="panel-content flex-1 overflow-y-auto custom-scrollbar">
        {activeDomain === 'air' && <AirDomainControls />}
        {activeDomain === 'land' && <LandDomainControls />}
        {activeDomain === 'sea' && <SeaDomainControls />}
        {activeDomain === 'space' && <SpaceDomainControls />}
      </div>
    </div>
  );
};

export default DomainTestPanel;
