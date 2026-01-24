# JaguarEngine Kubernetes Cloud-Burst Auto-Scaling

This directory contains Kubernetes manifests for deploying JaguarEngine with automatic horizontal scaling based on entity count, CPU, and memory utilization.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Kubernetes Cluster                        │
│                                                               │
│  ┌──────────────┐      ┌──────────────┐                     │
│  │ Prometheus   │◄─────┤ ServiceMonitor│                     │
│  │   Server     │      └──────────────┘                     │
│  └──────┬───────┘                                            │
│         │ scrape                                             │
│         ▼                                                     │
│  ┌──────────────────────────────────────────────────────┐   │
│  │        JaguarEngine Pods (1-100 replicas)             │   │
│  │  ┌─────┐  ┌─────┐  ┌─────┐              ┌─────┐      │   │
│  │  │ Pod │  │ Pod │  │ Pod │     ...       │ Pod │      │   │
│  │  │  1  │  │  2  │  │  3  │              │ 100 │      │   │
│  │  └─────┘  └─────┘  └─────┘              └─────┘      │   │
│  └──────────────────────────────────────────────────────┘   │
│         ▲                                     ▲               │
│         │                                     │               │
│  ┌──────┴────────┐                    ┌──────┴────────┐     │
│  │  Service      │                    │  Headless     │     │
│  │  (ClusterIP)  │                    │  Service      │     │
│  └───────────────┘                    └───────────────┘     │
│         ▲                                     ▲               │
│         │                                     │               │
│  ┌──────┴────────────────────────────────────┴──────┐       │
│  │    HorizontalPodAutoscaler (HPA)                  │       │
│  │    • CPU: 70%                                     │       │
│  │    • Memory: 75%                                  │       │
│  │    • Entity Count: 7000/pod                       │       │
│  │    • Min: 1, Max: 100 replicas                    │       │
│  └───────────────────────────────────────────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

## Files

| File | Description |
|------|-------------|
| `deployment.yaml` | Main deployment with pod specs, resource limits, health probes |
| `service.yaml` | Services for load balancing and peer discovery |
| `configmap.yaml` | Configuration parameters and thresholds |
| `hpa.yaml` | HorizontalPodAutoscaler with scaling policies |
| `prometheus-rules.yaml` | ServiceMonitor and alerting rules |

## Prerequisites

1. **Kubernetes Cluster** (v1.24+)
   - Minikube, GKE, EKS, AKS, or any k8s cluster

2. **Prometheus Operator** (for custom metrics)
   ```bash
   kubectl apply -f https://raw.githubusercontent.com/prometheus-operator/prometheus-operator/main/bundle.yaml
   ```

3. **Metrics Server** (for CPU/memory metrics)
   ```bash
   kubectl apply -f https://github.com/kubernetes-sigs/metrics-server/releases/latest/download/components.yaml
   ```

4. **Prometheus Adapter** (for custom metrics in HPA)
   ```bash
   helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
   helm install prometheus-adapter prometheus-community/prometheus-adapter
   ```

## Quick Start

### 1. Deploy JaguarEngine

```bash
# Create namespace (optional)
kubectl create namespace jaguar

# Apply all manifests
kubectl apply -f configmap.yaml
kubectl apply -f deployment.yaml
kubectl apply -f service.yaml
kubectl apply -f hpa.yaml
kubectl apply -f prometheus-rules.yaml
```

### 2. Verify Deployment

```bash
# Check pods
kubectl get pods -l app=jaguar-engine

# Check HPA status
kubectl get hpa jaguar-engine-hpa

# Check services
kubectl get svc -l app=jaguar-engine

# View pod logs
kubectl logs -l app=jaguar-engine --tail=50 -f
```

### 3. Monitor Scaling

```bash
# Watch HPA scale in real-time
kubectl get hpa jaguar-engine-hpa --watch

# Check current metrics
kubectl describe hpa jaguar-engine-hpa

# View pod metrics
kubectl top pods -l app=jaguar-engine
```

## Configuration

### Scaling Thresholds

Edit `configmap.yaml` to adjust scaling behavior:

```yaml
# Entity-based scaling
entity_scale_up_threshold: "8000"    # Scale up at 8000 entities/pod
entity_scale_down_threshold: "2000"  # Scale down at 2000 entities/pod

# CPU-based scaling
cpu_scale_up_threshold: "70"         # Scale up at 70% CPU
cpu_scale_down_threshold: "30"       # Scale down at 30% CPU

# Cooldown periods
scale_up_cooldown_seconds: "30"      # Wait 30s between scale-ups
scale_down_cooldown_seconds: "300"   # Wait 5m between scale-downs
```

### Resource Limits

Edit `deployment.yaml` to adjust pod resources:

```yaml
resources:
  requests:
    cpu: 1000m      # 1 CPU core
    memory: 2Gi     # 2 GB RAM
  limits:
    cpu: 4000m      # 4 CPU cores max
    memory: 8Gi     # 8 GB RAM max
```

### Replica Bounds

Edit `hpa.yaml` to adjust cluster size:

```yaml
minReplicas: 1    # Minimum pods
maxReplicas: 100  # Maximum pods
```

## Scaling Policies

### Scale-Up Behavior

- **Trigger**: CPU > 70%, Memory > 75%, or Entities > 7000
- **Rate**: Double replicas every 15 seconds (up to +10 pods/min)
- **Stabilization**: 30-second window to prevent thrashing

### Scale-Down Behavior

- **Trigger**: CPU < 30%, Memory < 40%, or Entities < 2000
- **Rate**: Remove 10% of replicas every 60 seconds (up to -5 pods/2min)
- **Stabilization**: 5-minute window to prevent premature scale-down

## Custom Metrics

JaguarEngine exports these Prometheus metrics:

| Metric | Type | Description |
|--------|------|-------------|
| `jaguar_entity_count` | Gauge | Current entity count per pod |
| `jaguar_simulation_tick_rate` | Gauge | Simulation tick rate (Hz) |
| `jaguar_simulation_frame_time_ms` | Histogram | Frame processing time |
| `jaguar_physics_computation_time_ms` | Histogram | Physics computation time |
| `jaguar_simulation_load` | Gauge | Composite load metric |
| `jaguar_entity_migration_success_total` | Counter | Successful migrations |
| `jaguar_entity_migration_failures_total` | Counter | Failed migrations |
| `jaguar_state_sync_lag_ms` | Gauge | State sync latency |

## Alerting

Prometheus alerts are configured for:

- **High entity count** (> 7500)
- **Critical entity count** (> 8000)
- **High CPU** (> 70%)
- **Critical CPU** (> 90%)
- **High memory** (> 75%)
- **Low tick rate** (< 30 Hz)
- **Pod not ready** (> 5 minutes)
- **High restart rate**
- **Cluster at min/max replicas**

View active alerts:
```bash
kubectl get prometheusrules
```

## Accessing Services

### Internal (from within cluster)

```bash
# Via load-balanced service
jaguar-engine.default.svc.cluster.local:50051

# Via headless service (direct pod access)
jaguar-engine-headless.default.svc.cluster.local:50051
```

### External (via LoadBalancer)

```bash
# Get external IP
kubectl get svc jaguar-engine-external

# Connect via gRPC
grpcurl -plaintext <EXTERNAL-IP>:50051 list
```

## Peer Discovery

Pods discover each other via Kubernetes API:

1. ServiceAccount with read permissions to pods/endpoints
2. Headless service publishes individual pod IPs
3. Each pod queries the headless service for peer list
4. Entity migration occurs between peers based on load

## Health Checks

| Probe | Endpoint | Purpose |
|-------|----------|---------|
| Liveness | `/health/live` | Restart pod if failing |
| Readiness | `/health/ready` | Remove from service if not ready |
| Startup | `/health/startup` | Wait up to 5 minutes for initialization |

## Troubleshooting

### HPA Not Scaling

```bash
# Check metrics server
kubectl top nodes
kubectl top pods

# Check HPA status
kubectl describe hpa jaguar-engine-hpa

# Check custom metrics
kubectl get --raw /apis/custom.metrics.k8s.io/v1beta1
```

### Pods Not Starting

```bash
# Check pod status
kubectl describe pod <pod-name>

# Check logs
kubectl logs <pod-name>

# Check events
kubectl get events --sort-by='.lastTimestamp'
```

### High Restart Rate

```bash
# Check resource limits
kubectl top pods -l app=jaguar-engine

# Check OOM kills
kubectl describe pod <pod-name> | grep -A 5 "Last State"

# Increase resource limits in deployment.yaml
```

### Migration Failures

```bash
# Check network connectivity
kubectl exec <pod-name> -- nc -zv jaguar-engine-headless 50051

# Check logs for migration errors
kubectl logs -l app=jaguar-engine | grep migration

# Verify RBAC permissions
kubectl auth can-i list pods --as=system:serviceaccount:default:jaguar-engine
```

## Performance Tuning

### For High Entity Count (>50k entities)

```yaml
# Increase resources
resources:
  requests:
    cpu: 2000m
    memory: 4Gi
  limits:
    cpu: 8000m
    memory: 16Gi

# Decrease scale-up threshold
entity_scale_up_threshold: "5000"
```

### For Low Latency (<10ms)

```yaml
# Increase tick rate
target_tick_rate: "120"

# Reduce batch sizes
entity_migration_batch_size: "50"
state_sync_interval_ms: "50"
```

### For Cost Optimization

```yaml
# Aggressive scale-down
scale_down_cooldown_seconds: "120"  # 2 minutes
entity_scale_down_threshold: "3000"

# Higher resource utilization targets
cpu_scale_up_threshold: "80"
memory_scale_up_threshold: "85"
```

## Advanced Features

### Vertical Pod Autoscaler (VPA)

VPA is included in `hpa.yaml` to automatically right-size resources:

```bash
# Install VPA (if not present)
git clone https://github.com/kubernetes/autoscaler.git
cd autoscaler/vertical-pod-autoscaler
./hack/vpa-up.sh

# VPA will automatically adjust resource requests
```

### Pod Disruption Budget (PDB)

Ensures at least 1 pod is always available during voluntary disruptions (node drains, upgrades):

```yaml
minAvailable: 1  # At least 1 pod must remain available
```

### Custom Metrics Adapter

Configure Prometheus Adapter to expose custom metrics:

```yaml
# prometheus-adapter-values.yaml
rules:
- seriesQuery: 'jaguar_entity_count'
  resources:
    template: <<.Resource>>
  name:
    matches: "^(.*)$"
    as: "jaguar_entity_count"
  metricsQuery: 'avg(<<.Series>>{<<.LabelMatchers>>})'
```

## Production Checklist

- [ ] Enable TLS for inter-pod communication
- [ ] Configure persistent volumes for state
- [ ] Set up log aggregation (ELK, Loki)
- [ ] Configure distributed tracing (Jaeger, Zipkin)
- [ ] Set up alertmanager for notifications
- [ ] Configure network policies for security
- [ ] Enable pod security policies
- [ ] Set up backup/restore procedures
- [ ] Configure ingress/gateway for external access
- [ ] Implement chaos testing
- [ ] Performance testing under load
- [ ] Disaster recovery plan

## References

- [Kubernetes HPA Documentation](https://kubernetes.io/docs/tasks/run-application/horizontal-pod-autoscale/)
- [Prometheus Operator](https://prometheus-operator.dev/)
- [Metrics Server](https://github.com/kubernetes-sigs/metrics-server)
- [Vertical Pod Autoscaler](https://github.com/kubernetes/autoscaler/tree/master/vertical-pod-autoscaler)
