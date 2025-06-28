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
- **Diagnostic Type**: ${payload.diagnosticType || 'hardware_diagnostic'}

## Executive Summary
${this.generateExecutiveSummary(payload)}

## System Health Summary
${payload.systemHealth ? this.formatSystemHealth(payload.systemHealth) : 'System health data not available'}

## Diagnostic Results

\`\`\`
${diagnosticResults.replace(/\|/g, '\n')}
\`\`\`

## Raw Diagnostic Messages
${payload.rawDiagnosticMessages ? this.formatRawDiagnosticMessages(payload.rawDiagnosticMessages) : 'No raw diagnostic messages captured'}

## Power Analysis
${payload.powerAnalysis ? this.formatPowerAnalysis(payload.powerAnalysis) : 'Power analysis data not available'}

## Hardware Status
${payload.hardwareStatus ? this.formatHardwareStatus(payload.hardwareStatus) : 'Hardware status data not available'}

## Session Log Entries
${payload.recentLogEntries ? this.formatLogEntries(payload.recentLogEntries) : 'No log entries available'}

## Recommendations
${payload.recommendations ? this.formatRecommendations(payload.recommendations) : 'No specific recommendations available'}

## Technical Details
${this.formatTechnicalDetails(payload)}

---
*Report generated by Freematics BLE Dashboard at ${timestamp}*
*Total log entries: ${payload.recentLogEntries?.length || 0} | Raw messages length: ${payload.rawDiagnosticMessages?.length || 0} characters*
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

    private generateExecutiveSummary(payload: any): string {
        const status = payload.diagnosticStatus || 'Unknown';
        const issues = payload.issuesFound || 0;
        const deviceHealth = this.assessDeviceHealth(payload);
        
        let summary = `**Status**: ${status}\n`;
        summary += `**Issues Detected**: ${issues}\n`;
        summary += `**Overall Health**: ${deviceHealth}\n\n`;
        
        if (status === 'SUCCESS' && issues === 0) {
            summary += '✅ All systems operating normally. No issues detected during diagnostic scan.';
        } else if (status === 'TIMEOUT') {
            summary += '⏰ Diagnostic timed out. Device may be unresponsive or experiencing connectivity issues.';
        } else if (issues > 0) {
            summary += `⚠️ ${issues} issue(s) detected. Review diagnostic details and recommendations below.`;
        } else {
            summary += '❓ Diagnostic completed with unknown status. Review details for more information.';
        }
        
        return summary;
    }

    private assessDeviceHealth(payload: any): string {
        const systemHealth = payload.systemHealth;
        const powerAnalysis = payload.powerAnalysis;
        const issues = payload.issuesFound || 0;
        
        if (issues === 0 && systemHealth?.batteryVoltage > 12.0) {
            return 'Excellent';
        } else if (issues <= 2 && systemHealth?.batteryVoltage > 11.5) {
            return 'Good';
        } else if (issues <= 5 || systemHealth?.batteryVoltage < 11.5) {
            return 'Fair';
        } else {
            return 'Poor';
        }
    }

    private formatRawDiagnosticMessages(rawMessages: string): string {
        if (!rawMessages || rawMessages.trim().length === 0) {
            return 'No raw diagnostic messages captured';
        }
        
        // Split by lines and format with proper markdown
        const lines = rawMessages.split('\n').filter(line => line.trim().length > 0);
        
        if (lines.length === 0) {
            return 'No diagnostic messages found';
        }
        
        return `\`\`\`
${rawMessages}
\`\`\`

**Message Summary**: ${lines.length} total messages captured during diagnostic session`;
    }

    private formatLogEntries(logEntries: any[]): string {
        if (!logEntries || !Array.isArray(logEntries) || logEntries.length === 0) {
            return 'No log entries available';
        }
        
        let formatted = `**Total Entries**: ${logEntries.length}\n\n`;
        
        // Group entries by type for better organization
        const groupedEntries = logEntries.reduce((groups, entry) => {
            const type = entry.type || 'data';
            if (!groups[type]) groups[type] = [];
            groups[type].push(entry);
            return groups;
        }, {});
        
        // Format each group
        Object.entries(groupedEntries).forEach(([type, entries]: [string, any[]]) => {
            formatted += `### ${type.toUpperCase()} Messages (${entries.length})\n\n`;
            formatted += '```\n';
            entries.slice(-10).forEach(entry => { // Show last 10 of each type
                formatted += `${entry.timestamp}: ${entry.message}\n`;
            });
            formatted += '```\n\n';
        });
        
        return formatted;
    }

    private formatTechnicalDetails(payload: any): string {
        let details = '';
        
        // Add payload structure information
        details += '### Payload Structure\n';
        details += `- **Device ID**: ${payload.deviceId || 'N/A'}\n`;
        details += `- **Device Name**: ${payload.deviceName || 'N/A'}\n`;
        details += `- **Diagnostic Type**: ${payload.diagnosticType || 'N/A'}\n`;
        details += `- **Timestamp**: ${payload.timestamp || 'N/A'}\n`;
        details += `- **Status**: ${payload.diagnosticStatus || 'N/A'}\n`;
        details += `- **Issues Found**: ${payload.issuesFound || 0}\n\n`;
        
        // Add data sizes
        details += '### Data Metrics\n';
        details += `- **Diagnostic Results Length**: ${payload.diagnosticResults?.length || 0} characters\n`;
        details += `- **Raw Messages Length**: ${payload.rawDiagnosticMessages?.length || 0} characters\n`;
        details += `- **Log Entries Count**: ${payload.recentLogEntries?.length || 0}\n`;
        details += `- **Recommendations Count**: ${Array.isArray(payload.recommendations) ? payload.recommendations.length : 0}\n\n`;
        
        // Add system information if available
        if (payload.systemHealth) {
            details += '### System Metrics\n';
            Object.entries(payload.systemHealth).forEach(([key, value]) => {
                details += `- **${key}**: ${value}\n`;
            });
            details += '\n';
        }
        
        // Add power information if available
        if (payload.powerAnalysis) {
            details += '### Power Metrics\n';
            Object.entries(payload.powerAnalysis).forEach(([key, value]) => {
                details += `- **${key}**: ${value}\n`;
            });
            details += '\n';
        }
        
        // Add hardware status if available
        if (payload.hardwareStatus) {
            details += '### Hardware Status\n';
            Object.entries(payload.hardwareStatus).forEach(([key, value]) => {
                details += `- **${key}**: ${value}\n`;
            });
            details += '\n';
        }
        
        return details;
    }
