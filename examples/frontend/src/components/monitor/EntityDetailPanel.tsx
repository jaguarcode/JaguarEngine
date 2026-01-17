import React from 'react';
import clsx from 'clsx';
import { useEntityStore, selectSelectedEntity } from '@/stores/entityStore';
import type { AircraftState, GroundVehicleState, ShipState, SpacecraftState } from '@/types';

// Helper to format numbers
const formatNum = (value: number, decimals = 2): string => {
  return value.toFixed(decimals);
};

// Property row component
const PropertyRow: React.FC<{
  label: string;
  value: string | number;
  unit?: string;
}> = ({ label, value, unit }) => (
  <div className="flex items-center justify-between py-1">
    <span className="text-xs text-gray-500">{label}</span>
    <span className="text-sm text-gray-200 font-mono">
      {typeof value === 'number' ? formatNum(value) : value}
      {unit && <span className="text-gray-500 ml-1">{unit}</span>}
    </span>
  </div>
);

// Section component
const Section: React.FC<{
  title: string;
  children: React.ReactNode;
}> = ({ title, children }) => (
  <div className="mb-4">
    <h4 className="text-xs font-semibold text-gray-500 uppercase mb-2">{title}</h4>
    <div className="space-y-0.5">{children}</div>
  </div>
);

// Aircraft details
const AircraftDetails: React.FC<{ entity: AircraftState }> = ({ entity }) => (
  <>
    <Section title="Flight Parameters">
      <PropertyRow label="Airspeed (TAS)" value={entity.airspeed} unit="m/s" />
      <PropertyRow label="Mach" value={entity.machNumber} />
      <PropertyRow label="Altitude" value={entity.altitude} unit="m MSL" />
      <PropertyRow label="Vertical Speed" value={entity.verticalSpeed} unit="m/s" />
      <PropertyRow label="Heading" value={entity.heading} unit="°" />
    </Section>

    <Section title="Flight Dynamics">
      <PropertyRow label="AoA" value={entity.aoa} unit="°" />
      <PropertyRow label="Sideslip" value={entity.sideslip} unit="°" />
      <PropertyRow label="G-Load" value={entity.gForce} unit="G" />
    </Section>

    <Section title="Controls">
      <PropertyRow label="Elevator" value={entity.elevator} />
      <PropertyRow label="Aileron" value={entity.aileron} />
      <PropertyRow label="Rudder" value={entity.rudder} />
      <PropertyRow label="Throttle" value={(entity.throttle * 100).toFixed(0)} unit="%" />
    </Section>

    <Section title="Systems">
      <PropertyRow label="Fuel" value={entity.fuel.toFixed(0)} unit="%" />
      <PropertyRow label="Gear" value={entity.gearDown ? 'DOWN' : 'UP'} />
      <PropertyRow label="Flaps" value={entity.flapsPosition} unit="°" />
    </Section>
  </>
);

// Ground vehicle details
const VehicleDetails: React.FC<{ entity: GroundVehicleState }> = ({ entity }) => (
  <>
    <Section title="Movement">
      <PropertyRow label="Speed" value={entity.speed} unit="m/s" />
      <PropertyRow label="Heading" value={entity.heading} unit="°" />
    </Section>

    <Section title="Controls">
      <PropertyRow label="Throttle" value={(entity.throttle * 100).toFixed(0)} unit="%" />
      <PropertyRow label="Steering" value={entity.steering} />
      <PropertyRow label="Brake" value={(entity.brake * 100).toFixed(0)} unit="%" />
    </Section>

    <Section title="Engine">
      <PropertyRow label="RPM" value={entity.engineRPM.toFixed(0)} />
      <PropertyRow label="Gear" value={entity.gear} />
      <PropertyRow label="Fuel" value={entity.fuel.toFixed(0)} unit="%" />
    </Section>

    <Section title="Terrain">
      <PropertyRow label="Ground Contact" value={entity.groundContact ? 'Yes' : 'No'} />
      <PropertyRow label="Sinkage" value={entity.sinkage * 100} unit="cm" />
    </Section>
  </>
);

// Ship details
const ShipDetails: React.FC<{ entity: ShipState }> = ({ entity }) => (
  <>
    <Section title="Navigation">
      <PropertyRow label="Speed" value={entity.speed} unit="m/s" />
      <PropertyRow label="Heading" value={entity.heading} unit="°" />
    </Section>

    <Section title="Motion">
      <PropertyRow label="Roll" value={entity.roll} unit="°" />
      <PropertyRow label="Pitch" value={entity.pitch} unit="°" />
      <PropertyRow label="Heave" value={entity.heave} unit="m" />
    </Section>

    <Section title="Controls">
      <PropertyRow label="Rudder" value={entity.rudder} />
      <PropertyRow label="Throttle" value={(entity.throttle * 100).toFixed(0)} unit="%" />
    </Section>

    <Section title="Hull">
      <PropertyRow label="Draft" value={entity.draft} unit="m" />
      <PropertyRow label="Displacement" value={entity.displacement} unit="t" />
    </Section>

    <Section title="Sea State">
      <PropertyRow label="Wave Height" value={entity.waveHeight} unit="m" />
      <PropertyRow label="Sea State" value={entity.seaState} />
    </Section>
  </>
);

// Spacecraft details
const SpacecraftDetails: React.FC<{ entity: SpacecraftState }> = ({ entity }) => (
  <>
    <Section title="Orbital Elements">
      <PropertyRow label="Semi-Major Axis" value={entity.semiMajorAxis} unit="km" />
      <PropertyRow label="Eccentricity" value={entity.eccentricity} />
      <PropertyRow label="Inclination" value={entity.inclination} unit="°" />
      <PropertyRow label="RAAN" value={entity.raan} unit="°" />
      <PropertyRow label="Arg. of Perigee" value={entity.argOfPerigee} unit="°" />
      <PropertyRow label="True Anomaly" value={entity.trueAnomaly} unit="°" />
    </Section>

    <Section title="Derived">
      <PropertyRow label="Altitude" value={entity.altitude} unit="km" />
      <PropertyRow label="Velocity" value={entity.velocity} unit="km/s" />
      <PropertyRow label="Period" value={entity.period} unit="min" />
    </Section>

    <Section title="Systems">
      <PropertyRow label="Power" value={entity.power.toFixed(0)} unit="%" />
      <PropertyRow label="Fuel" value={entity.fuel.toFixed(0)} unit="%" />
    </Section>
  </>
);

export const EntityDetailPanel: React.FC = () => {
  const entity = useEntityStore(selectSelectedEntity);

  if (!entity) {
    return (
      <div className="panel h-full flex items-center justify-center">
        <div className="text-center text-gray-500">
          <div className="text-4xl mb-2">📍</div>
          <div className="text-sm">Select an entity to view details</div>
        </div>
      </div>
    );
  }

  return (
    <div className="panel h-full flex flex-col">
      <div className="panel-header">
        <div className="flex items-center gap-2">
          <div
            className={clsx('status-dot', {
              'status-dot-active': entity.isActive,
              'status-dot-idle': !entity.isActive,
            })}
          />
          <h3 className="panel-title truncate">{entity.name}</h3>
        </div>
        <span
          className={clsx('domain-badge', {
            'domain-badge-air': entity.domain === 'air',
            'domain-badge-land': entity.domain === 'land',
            'domain-badge-sea': entity.domain === 'sea',
            'domain-badge-space': entity.domain === 'space',
          })}
        >
          {entity.domain}
        </span>
      </div>

      <div className="panel-content flex-1 overflow-y-auto custom-scrollbar">
        {/* Common info */}
        <Section title="Identification">
          <PropertyRow label="ID" value={entity.id} />
          <PropertyRow label="Kind" value={entity.kind} />
          <PropertyRow
            label="Entity ID"
            value={`${entity.identifier.siteId}:${entity.identifier.applicationId}:${entity.identifier.entityNumber}`}
          />
        </Section>

        <Section title="Position">
          <PropertyRow label="Latitude" value={entity.position.latitude} unit="°" />
          <PropertyRow label="Longitude" value={entity.position.longitude} unit="°" />
          <PropertyRow label="Altitude" value={entity.position.altitude} unit="m" />
        </Section>

        <Section title="Velocity">
          <PropertyRow label="North" value={entity.velocity.north} unit="m/s" />
          <PropertyRow label="East" value={entity.velocity.east} unit="m/s" />
          <PropertyRow label="Down" value={entity.velocity.down} unit="m/s" />
        </Section>

        <Section title="Status">
          <PropertyRow label="Health" value={entity.health.toFixed(0)} unit="%" />
          <PropertyRow label="Damage" value={entity.damage.toFixed(0)} unit="%" />
        </Section>

        {/* Domain-specific details */}
        {entity.domain === 'air' && entity.kind === 'platform' && (
          <AircraftDetails entity={entity as AircraftState} />
        )}
        {entity.domain === 'land' && entity.kind === 'platform' && (
          <VehicleDetails entity={entity as GroundVehicleState} />
        )}
        {entity.domain === 'sea' && entity.kind === 'platform' && (
          <ShipDetails entity={entity as ShipState} />
        )}
        {entity.domain === 'space' && (
          <SpacecraftDetails entity={entity as unknown as SpacecraftState} />
        )}

        {/* Custom properties */}
        {Object.keys(entity.properties).length > 0 && (
          <Section title="Properties">
            {Object.entries(entity.properties).map(([key, value]) => (
              <PropertyRow key={key} label={key} value={String(value)} />
            ))}
          </Section>
        )}
      </div>
    </div>
  );
};

export default EntityDetailPanel;
