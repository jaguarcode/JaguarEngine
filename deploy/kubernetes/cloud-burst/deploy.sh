#!/bin/bash
# JaguarEngine Kubernetes Cloud-Burst Deployment Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
NAMESPACE="${NAMESPACE:-default}"
ENVIRONMENT="${ENVIRONMENT:-production}"
IMAGE_TAG="${IMAGE_TAG:-latest}"

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# Check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"

    # Check kubectl
    if ! command -v kubectl &> /dev/null; then
        print_error "kubectl not found! Please install kubectl first."
        exit 1
    fi
    print_success "kubectl found: $(kubectl version --client --short 2>/dev/null | head -n1)"

    # Check cluster connection
    if ! kubectl cluster-info &> /dev/null; then
        print_error "Cannot connect to Kubernetes cluster!"
        exit 1
    fi
    print_success "Connected to Kubernetes cluster"

    # Check metrics-server
    if ! kubectl get deployment metrics-server -n kube-system &> /dev/null; then
        print_warning "metrics-server not found. HPA may not work correctly."
        read -p "Install metrics-server? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            install_metrics_server
        fi
    else
        print_success "metrics-server is installed"
    fi

    # Check prometheus-operator
    if ! kubectl get crd servicemonitors.monitoring.coreos.com &> /dev/null; then
        print_warning "Prometheus Operator not found. Custom metrics won't work."
        read -p "Install Prometheus Operator? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            install_prometheus_operator
        fi
    else
        print_success "Prometheus Operator is installed"
    fi
}

# Install metrics-server
install_metrics_server() {
    print_info "Installing metrics-server..."
    kubectl apply -f https://github.com/kubernetes-sigs/metrics-server/releases/latest/download/components.yaml
    print_success "metrics-server installed"
}

# Install Prometheus Operator
install_prometheus_operator() {
    print_info "Installing Prometheus Operator..."
    kubectl apply -f https://raw.githubusercontent.com/prometheus-operator/prometheus-operator/main/bundle.yaml
    print_success "Prometheus Operator installed"
}

# Create namespace if it doesn't exist
create_namespace() {
    if ! kubectl get namespace "$NAMESPACE" &> /dev/null; then
        print_info "Creating namespace: $NAMESPACE"
        kubectl create namespace "$NAMESPACE"
        print_success "Namespace created: $NAMESPACE"
    else
        print_info "Namespace already exists: $NAMESPACE"
    fi
}

# Deploy based on environment
deploy() {
    print_header "Deploying JaguarEngine ($ENVIRONMENT)"

    create_namespace

    if [[ -d "overlays/$ENVIRONMENT" ]]; then
        print_info "Using Kustomize overlay for $ENVIRONMENT"
        kubectl apply -k "overlays/$ENVIRONMENT"
    else
        print_info "Using base manifests"
        kubectl apply -f configmap.yaml -n "$NAMESPACE"
        kubectl apply -f deployment.yaml -n "$NAMESPACE"
        kubectl apply -f service.yaml -n "$NAMESPACE"
        kubectl apply -f hpa.yaml -n "$NAMESPACE"
        kubectl apply -f prometheus-rules.yaml -n "$NAMESPACE"
    fi

    print_success "Deployment applied successfully"
}

# Wait for deployment
wait_for_deployment() {
    print_header "Waiting for Deployment"

    print_info "Waiting for deployment to be ready..."
    kubectl rollout status deployment/jaguar-engine -n "$NAMESPACE" --timeout=5m

    print_success "Deployment is ready!"
}

# Show status
show_status() {
    print_header "Deployment Status"

    echo -e "${BLUE}Pods:${NC}"
    kubectl get pods -l app=jaguar-engine -n "$NAMESPACE"

    echo -e "\n${BLUE}HPA:${NC}"
    kubectl get hpa -n "$NAMESPACE"

    echo -e "\n${BLUE}Services:${NC}"
    kubectl get svc -l app=jaguar-engine -n "$NAMESPACE"

    echo -e "\n${BLUE}Recent Events:${NC}"
    kubectl get events -n "$NAMESPACE" --sort-by='.lastTimestamp' | tail -10
}

# Main deployment function
main() {
    print_header "JaguarEngine Kubernetes Cloud-Burst Deployment"
    echo "Environment: $ENVIRONMENT"
    echo "Namespace: $NAMESPACE"
    echo "Image Tag: $IMAGE_TAG"
    echo

    # Confirm deployment
    read -p "Continue with deployment? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Deployment cancelled"
        exit 0
    fi

    # Run deployment steps
    check_prerequisites
    deploy
    wait_for_deployment
    show_status

    print_header "Deployment Complete!"
    print_info "To watch the HPA and pods:"
    echo "  kubectl get hpa -n $NAMESPACE --watch"
    echo
    print_info "To view logs:"
    echo "  kubectl logs -l app=jaguar-engine -n $NAMESPACE -f"
    echo
    print_info "To port-forward:"
    echo "  kubectl port-forward -n $NAMESPACE svc/jaguar-engine 50051:50051"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -e|--environment)
            ENVIRONMENT="$2"
            shift 2
            ;;
        -n|--namespace)
            NAMESPACE="$2"
            shift 2
            ;;
        -t|--tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -e, --environment ENV   Deployment environment (development, staging, production)"
            echo "  -n, --namespace NS      Kubernetes namespace (default: default)"
            echo "  -t, --tag TAG           Docker image tag (default: latest)"
            echo "  -h, --help              Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 -e production -n jaguar-prod -t v1.0.0"
            echo "  $0 -e development"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Run main
main
