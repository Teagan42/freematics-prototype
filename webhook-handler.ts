    async handleWebhook(payload: any): Promise<void> {
        return this.tracer.startActiveSpan('handleWebhook', async (span) => {
            try {
                span.setAttribute('webhook.type', 'FreematicsDiagnostic');
                span.setAttribute('webhook.event', payload?.diagnosticType || 'hardware_diagnostic');
                span.setAttribute('device.id', payload?.deviceId || 'unknown');
                span.setAttribute('device.name', payload?.deviceName || 'FreematicsCustom');

                // Validate the payload using the secret token
                const isValid = this.validateDiagnosticPayload(payload);
                span.setAttribute('webhook.signature_valid', isValid);

                if (!isValid) {
                    this.logger.error('Invalid diagnostic webhook payload received');
                    span.setAttribute('error', true);
                    span.setAttribute('error.type', 'payload_validation_failed');

                    try {
                        const note = this.convertDiagnosticToMarkdown(payload);
                        const Note = await this.prismaService.note.create({
                            data: {
                                name: `Invalid Diagnostic - ${new Date().toISOString()}`,
                                content: note,
                            },
                        });
                        span.setAttribute('note.created', true);
                        span.setAttribute('note.id', Note.id);
                    } catch (error) {
                        span.recordException(error);
                        console.dir(error);
                    }

                    const error = new Error('Diagnostic webhook payload validation failed');
                    span.recordException(error);
                    span.setStatus({ code: 2 }); // Error status
                    throw error;
                }

                // Process diagnostic data
                this.logger.log('Received and validated diagnostic webhook payload');
                span.setAttribute('webhook.processing', true);
                span.setAttribute('diagnostic.timestamp', payload?.timestamp);
                span.setAttribute('diagnostic.status', payload?.diagnosticStatus || 'unknown');
                span.setAttribute('diagnostic.issues_found', payload?.issuesFound || 0);

                try {
                    const diagnosticMarkdown = this.convertDiagnosticToMarkdown(payload);
                    span.setAttribute('diagnostic.content_length', diagnosticMarkdown.length);

                    const diagnosticName = this.generateDiagnosticName(payload);
                    const Note = await this.prismaService.note.create({
                        data: {
                            name: diagnosticName,
                            content: diagnosticMarkdown,
                        },
                    });

                    span.setAttribute('note.created', true);
                    span.setAttribute('note.id', Note.id);
                    span.setAttribute('note.name', Note.name);
                    this.logger.debug(`Created diagnostic note with ID: ${Note.id}`);
                } catch (error) {
                    span.recordException(error);
                    span.setAttribute('note.created', false);
                    span.setAttribute('error', true);
                    span.setAttribute('error.type', 'diagnostic_note_creation_failed');
                    this.logger.error('Failed to create diagnostic note from webhook payload', error);
                    console.dir(error);
                }

                span.setStatus({ code: 1 }); // Success status
            } catch (error) {
                span.recordException(error);
                span.setStatus({ code: 2 }); // Error status
                throw error;
            } finally {
                span.end();
            }
        });
    }

    private validateDiagnosticPayload(payload: any): boolean {
        // Validate required fields for Freematics diagnostic data
        if (!payload) return false;
        
        // Check for required diagnostic fields
        const hasRequiredFields = payload.deviceId && 
                                 payload.timestamp && 
                                 payload.diagnosticResults;
        
        // Validate diagnostic results structure
        const hasValidResults = payload.diagnosticResults && 
                               typeof payload.diagnosticResults === 'string' &&
                               payload.diagnosticResults.length > 0;
        
        return hasRequiredFields && hasValidResults;
    }

    private generateDiagnosticName(payload: any): string {
        const timestamp = new Date(payload.timestamp || Date.now()).toISOString();
        const deviceId = payload.deviceId || 'unknown';
        const status = payload.diagnosticStatus || 'unknown';
        const issueCount = payload.issuesFound || 0;
        
        return `Freematics Diagnostic - ${deviceId} - ${status} (${issueCount} issues) - ${timestamp}`;
    }

    private convertDiagnosticToMarkdown(payload: any): string {
        const timestamp = new Date(payload.timestamp || Date.now()).toISOString();
        const deviceId = payload.deviceId || 'Unknown Device';
        const deviceName = payload.deviceName || 'FreematicsCustom';
        const diagnosticStatus = payload.diagnosticStatus || 'Unknown';
        const issuesFound = payload.issuesFound || 0;
        const diagnosticResults = payload.diagnosticResults || 'No diagnostic data available';

        return `# Freematics Hardware Diagnostic Report

## Device Information
- **Device ID**: ${deviceId}
- **Device Name**: ${deviceName}
- **Timestamp**: ${timestamp}
- **Diagnostic Status**: ${diagnosticStatus}
- **Issues Found**: ${issuesFound}

## System Health Summary
${payload.systemHealth ? this.formatSystemHealth(payload.systemHealth) : 'System health data not available'}

## Diagnostic Results

\`\`\`
${diagnosticResults.replace(/\|/g, '\n')}
\`\`\`

## Power Analysis
${payload.powerAnalysis ? this.formatPowerAnalysis(payload.powerAnalysis) : 'Power analysis data not available'}

## Hardware Status
${payload.hardwareStatus ? this.formatHardwareStatus(payload.hardwareStatus) : 'Hardware status data not available'}

## Recommendations
${payload.recommendations ? this.formatRecommendations(payload.recommendations) : 'No specific recommendations available'}

---
*Report generated by Freematics BLE Dashboard at ${timestamp}*
`;
    }

    private formatSystemHealth(systemHealth: any): string {
        if (!systemHealth) return 'No system health data';
        
        let health = '';
        if (systemHealth.engineLoad !== undefined) {
            health += `- **Engine Load**: ${systemHealth.engineLoad}% ${systemHealth.engineLoad > 85 ? '⚠️' : '✅'}\n`;
        }
        if (systemHealth.coolantTemp !== undefined) {
            health += `- **Coolant Temperature**: ${systemHealth.coolantTemp}°F ${systemHealth.coolantTemp > 220 ? '🔥' : '✅'}\n`;
        }
        if (systemHealth.batteryVoltage !== undefined) {
            health += `- **Battery Voltage**: ${systemHealth.batteryVoltage}V ${systemHealth.batteryVoltage < 12.0 ? '🔋' : '✅'}\n`;
        }
        if (systemHealth.fuelLevel !== undefined) {
            health += `- **Fuel Level**: ${systemHealth.fuelLevel}% ${systemHealth.fuelLevel < 25 ? '⛽' : '✅'}\n`;
        }
        
        return health || 'System health metrics not available';
    }

    private formatPowerAnalysis(powerAnalysis: any): string {
        if (!powerAnalysis) return 'No power analysis data';
        
        return `- **Input Voltage**: ${powerAnalysis.inputVoltage || 'N/A'}V
- **Power Source**: ${powerAnalysis.powerSource || 'Unknown'}
- **Voltage Status**: ${powerAnalysis.voltageStatus || 'Unknown'}
- **3.3V Rail**: ${powerAnalysis.rail3v3 || 'Unknown'}`;
    }

    private formatHardwareStatus(hardwareStatus: any): string {
        if (!hardwareStatus) return 'No hardware status data';
        
        let status = '';
        if (hardwareStatus.bleStatus) {
            status += `- **BLE**: ${hardwareStatus.bleStatus}\n`;
        }
        if (hardwareStatus.obdStatus) {
            status += `- **OBD-II**: ${hardwareStatus.obdStatus}\n`;
        }
        if (hardwareStatus.gpsStatus) {
            status += `- **GPS**: ${hardwareStatus.gpsStatus}\n`;
        }
        if (hardwareStatus.storageStatus) {
            status += `- **Storage**: ${hardwareStatus.storageStatus}\n`;
        }
        
        return status || 'Hardware status not available';
    }

    private formatRecommendations(recommendations: string[]): string {
        if (!recommendations || !Array.isArray(recommendations)) {
            return 'No specific recommendations';
        }
        
        return recommendations.map(rec => `- ${rec}`).join('\n');
    }
