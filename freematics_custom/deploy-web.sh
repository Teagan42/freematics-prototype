#!/bin/bash

# Freematics BLE Dashboard Web Deployment Script
# Deploys the HTML dashboard to a remote nginx server

set -e

echo "🌐 Freematics BLE Dashboard Web Deployment"
echo "=========================================="

# Configuration
LOCAL_HTML_FILE="dashboard.html"
REMOTE_HTML_FILE="ble.html"
REMOTE_PATH="/var/www/html"
DEFAULT_USER="deploy"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS] <server>"
    echo ""
    echo "Options:"
    echo "  -u, --user USER     SSH username (default: $DEFAULT_USER)"
    echo "  -p, --port PORT     SSH port (default: 22)"
    echo "  -k, --key KEYFILE   SSH private key file"
    echo "  -t, --test          Test connection only, don't deploy"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 example.com"
    echo "  $0 -u ubuntu 192.168.1.100"
    echo "  $0 --user root --port 2222 myserver.com"
    echo "  $0 --test example.com"
    echo ""
}

# Parse command line arguments
SSH_USER="$DEFAULT_USER"
SSH_PORT="22"
SSH_KEY=""
TEST_ONLY=false
SERVER=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -u|--user)
            SSH_USER="$2"
            shift 2
            ;;
        -p|--port)
            SSH_PORT="$2"
            shift 2
            ;;
        -k|--key)
            SSH_KEY="$2"
            shift 2
            ;;
        -t|--test)
            TEST_ONLY=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        -*)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
        *)
            if [ -z "$SERVER" ]; then
                SERVER="$1"
            else
                print_error "Multiple servers specified: $SERVER and $1"
                show_usage
                exit 1
            fi
            shift
            ;;
    esac
done

# Validate required arguments
if [ -z "$SERVER" ]; then
    print_error "Server address is required"
    show_usage
    exit 1
fi

# Build SSH command options
SSH_OPTS="-p $SSH_PORT"
if [ -n "$SSH_KEY" ]; then
    if [ ! -f "$SSH_KEY" ]; then
        print_error "SSH key file not found: $SSH_KEY"
        exit 1
    fi
    SSH_OPTS="$SSH_OPTS -i $SSH_KEY"
fi

SSH_TARGET="$SSH_USER@$SERVER"

print_info "Deployment Configuration:"
echo "  Server: $SERVER"
echo "  User: $SSH_USER"
echo "  Port: $SSH_PORT"
echo "  Local file: $LOCAL_HTML_FILE"
echo "  Remote file: $REMOTE_PATH/$REMOTE_HTML_FILE"
if [ -n "$SSH_KEY" ]; then
    echo "  SSH Key: $SSH_KEY"
fi
echo ""

# Check if local HTML file exists
check_local_file() {
    # Try to find the HTML file in current directory or freematics_custom subdirectory
    if [ -f "$LOCAL_HTML_FILE" ]; then
        print_status "Local HTML file found: $LOCAL_HTML_FILE"
    elif [ -f "freematics_custom/$LOCAL_HTML_FILE" ]; then
        LOCAL_HTML_FILE="freematics_custom/$LOCAL_HTML_FILE"
        print_status "Local HTML file found: $LOCAL_HTML_FILE"
    else
        print_error "Local HTML file not found: $LOCAL_HTML_FILE"
        print_info "Searched in:"
        echo "  - Current directory: ./$LOCAL_HTML_FILE"
        echo "  - Freematics directory: ./freematics_custom/$LOCAL_HTML_FILE"
        print_info "Make sure the dashboard.html file exists in one of these locations"
        exit 1
    fi
}

# Test SSH connection
test_ssh_connection() {
    print_info "Testing SSH connection to $SSH_TARGET..."
    
    if ssh $SSH_OPTS -o ConnectTimeout=10 -o BatchMode=yes "$SSH_TARGET" "echo 'SSH connection successful'" 2>/dev/null; then
        print_status "SSH connection successful"
        return 0
    else
        print_error "SSH connection failed"
        print_info "Please check:"
        echo "  1. Server address and port are correct"
        echo "  2. SSH keys are properly configured"
        echo "  3. User has appropriate permissions"
        echo "  4. Server is accessible from your network"
        return 1
    fi
}

# Check remote permissions
check_remote_permissions() {
    print_info "Checking remote permissions..."
    
    # Check if remote directory exists and is writable
    if ssh $SSH_OPTS "$SSH_TARGET" "test -d $REMOTE_PATH && test -w $REMOTE_PATH" 2>/dev/null; then
        print_status "Remote directory is accessible and writable"
        return 0
    else
        print_warning "Remote directory may not be writable"
        print_info "Attempting to create/fix permissions..."
        
        # Try to create directory and set permissions
        if ssh $SSH_OPTS "$SSH_TARGET" "sudo mkdir -p $REMOTE_PATH && sudo chown $SSH_USER:$SSH_USER $REMOTE_PATH" 2>/dev/null; then
            print_status "Remote directory permissions fixed"
            return 0
        else
            print_error "Cannot access or create remote directory: $REMOTE_PATH"
            print_info "You may need to:"
            echo "  1. Create the directory: sudo mkdir -p $REMOTE_PATH"
            echo "  2. Set ownership: sudo chown $SSH_USER:$SSH_USER $REMOTE_PATH"
            echo "  3. Set permissions: sudo chmod 755 $REMOTE_PATH"
            return 1
        fi
    fi
}

# Deploy the HTML file
deploy_html() {
    print_info "Deploying HTML file..."
    
    print_info "File details:"
    echo "  Local file: $LOCAL_HTML_FILE"
    echo "  File size: $(wc -c < "$LOCAL_HTML_FILE") bytes"
    echo "  Target: $SSH_TARGET:$REMOTE_PATH/$REMOTE_HTML_FILE"
    
    # Upload the file directly (no temp file needed)
    print_info "Uploading file..."
    if scp $SSH_OPTS "$LOCAL_HTML_FILE" "$SSH_TARGET:$REMOTE_PATH/$REMOTE_HTML_FILE"; then
        print_status "HTML file uploaded successfully"
        
        # Set appropriate permissions
        if ssh $SSH_OPTS "$SSH_TARGET" "chmod 644 $REMOTE_PATH/$REMOTE_HTML_FILE"; then
            print_status "File permissions set correctly"
        else
            print_warning "Could not set file permissions (file may still work)"
        fi
        
        return 0
    else
        print_error "Failed to upload HTML file"
        print_info "Troubleshooting steps:"
        echo "  1. Check if remote directory exists: ssh $SSH_USER@$SERVER 'ls -la $REMOTE_PATH'"
        echo "  2. Check disk space: ssh $SSH_USER@$SERVER 'df -h $REMOTE_PATH'"
        echo "  3. Test manual upload: scp $LOCAL_HTML_FILE $SSH_USER@$SERVER:/tmp/"
        return 1
    fi
}

# Verify deployment
verify_deployment() {
    print_info "Verifying deployment..."
    
    # Check if file exists and get its size
    REMOTE_SIZE=$(ssh $SSH_OPTS "$SSH_TARGET" "stat -c%s $REMOTE_PATH/$REMOTE_HTML_FILE 2>/dev/null || echo 0")
    LOCAL_SIZE=$(stat -f%z "$LOCAL_HTML_FILE" 2>/dev/null || stat -c%s "$LOCAL_HTML_FILE" 2>/dev/null || echo 0)
    
    if [ "$REMOTE_SIZE" -eq "$LOCAL_SIZE" ] && [ "$REMOTE_SIZE" -gt 0 ]; then
        print_status "Deployment verified (file sizes match: $LOCAL_SIZE bytes)"
        return 0
    else
        print_error "Deployment verification failed"
        echo "  Local size: $LOCAL_SIZE bytes"
        echo "  Remote size: $REMOTE_SIZE bytes"
        return 1
    fi
}

# Get server info for final message
get_server_info() {
    print_info "Getting server information..."
    
    # Try to get server's public IP or hostname
    SERVER_INFO=$(ssh $SSH_OPTS "$SSH_TARGET" "curl -s ifconfig.me 2>/dev/null || hostname -f 2>/dev/null || echo '$SERVER'" 2>/dev/null || echo "$SERVER")
    
    echo ""
    print_status "Deployment completed successfully!"
    echo ""
    echo "🌐 Your Freematics BLE Dashboard is now available at:"
    echo "   http://$SERVER_INFO/$REMOTE_HTML_FILE"
    echo "   https://$SERVER_INFO/$REMOTE_HTML_FILE (if SSL is configured)"
    echo ""
    echo "📱 To use the dashboard:"
    echo "   1. Open the URL in Chrome, Edge, or another Web Bluetooth compatible browser"
    echo "   2. Click 'Connect to Freematics' button"
    echo "   3. Select your Freematics device from the Bluetooth pairing dialog"
    echo "   4. Monitor real-time vehicle data"
    echo ""
}

# Main deployment process
main() {
    echo "Starting deployment process..."
    echo ""
    
    # Pre-flight checks
    check_local_file
    
    if ! test_ssh_connection; then
        exit 1
    fi
    
    if [ "$TEST_ONLY" = true ]; then
        print_status "Connection test completed successfully"
        exit 0
    fi
    
    if ! check_remote_permissions; then
        exit 1
    fi
    
    # Deploy
    if ! deploy_html; then
        exit 1
    fi
    
    if ! verify_deployment; then
        print_warning "Deployment may have issues, but file was uploaded"
    fi
    
    # Success message
    get_server_info
}

# Handle script interruption
trap 'echo ""; print_warning "Deployment interrupted"; exit 1' INT TERM

# Run main function
main "$@"
