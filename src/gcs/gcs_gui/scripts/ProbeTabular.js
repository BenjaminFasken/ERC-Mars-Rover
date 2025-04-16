importPackage(org.csstudio.opibuilder.scriptUtil);
importPackage(org.csstudio.swt.widgets.natives);

function updateProbeTable(pvs, widget) {
    var table = widget.getTable();

    // Configure columns (run once)
    if (table.getColumnCount() < 5) {
        table.setColumnHeaders(Java.to([
            "Probe ID",
            "X (m)",
            "Y (m)",
            "Z (m)",
            "Confidence"
        ], "java.lang.String[]"));
        table.setColumnWidths(Java.to([100, 100, 100, 100, 100], "int[]"));
        for (var i = 0; i < 5; i++) {
            table.setColumnEditable(i, false);
        }
        table.autoSizeColumns();
    }

    // Log column count
    //ConsoleUtil.writeInfo("Table has " + table.getColumnCount() + " columns");

    // Validate PVs
    var pvPositions = pvs[0];
    var pvConfidences = pvs[1];
    if (!pvPositions || !pvConfidences) {
        ConsoleUtil.writeInfo("Error: Missing PVs - Positions=" + pvPositions + ", Confidences=" + pvConfidences);
        var desiredRows = 1;
        while (table.getRowCount() > desiredRows) {
            table.deleteRow(table.getRowCount() - 1);
        }
        while (table.getRowCount() < desiredRows) {
            var rowIndex = table.appendRow();
            table.setCellText(rowIndex, 0, "0");
            for (var col = 1; col < 5; col++) {
                table.setCellText(rowIndex, col, "-");
            }
        }
        table.refresh();
        return;
    }

    // Collect valid probes
    var validProbes = [];
    try {
        if (pvPositions.isConnected() && pvConfidences.isConnected()) {
            var confidences = PVUtil.getDoubleArray(pvConfidences);
            var positionsArray = pvPositions.getValue();

            if (confidences && positionsArray) {
                //ConsoleUtil.writeInfo("positionsArray type: " + positionsArray.getClass().getName());
                var positionsList = positionsArray.getData();
                if (!positionsList) {
                    ConsoleUtil.writeInfo("Error: positionsArray.getData() returned null");
                    return;
                }

                var confidencesStr = "[";
                for (var j = 0; j < confidences.length; j++) {
                    confidencesStr += confidences[j].toFixed(2);
                    if (j < confidences.length - 1) confidencesStr += ", ";
                }
                confidencesStr += "]";
                //ConsoleUtil.writeInfo("Confidences: " + confidencesStr);

                var numProbes = Math.min(confidences.length, positionsList.length, 3);
                //ConsoleUtil.writeInfo("Number of probes to process: " + numProbes);
                for (var i = 0; i < numProbes; i++) {
                    var confidence = confidences[i];
                    if (confidence > 0) {
                        var pos = positionsList[i];
                        //ConsoleUtil.writeInfo("Probe " + i + " pos type: " + pos.getClass().getName());

                        // Handle pos if it's a String
                        var x, y, z;
                        if (typeof pos === "string" || pos instanceof java.lang.String) {
                            try {
                                // Attempt to parse as JSON
                                var parsed;
                                try {
                                    parsed = JSON.parse(pos);
                                    x = parsed.x;
                                    y = parsed.y;
                                    z = parsed.z;
                                } catch (jsonErr) {
                                    // If JSON parsing fails, try splitting as space-separated values
                                    var coords = pos.trim().split(/\s+/);
                                    if (coords.length >= 3) {
                                        x = parseFloat(coords[0]);
                                        y = parseFloat(coords[1]);
                                        z = parseFloat(coords[2]);
                                    } else {
                                        throw new Error("Invalid position format: " + pos);
                                    }
                                }
                            } catch (parseErr) {
                                ConsoleUtil.writeInfo("Error parsing probe " + i + " position string: " + parseErr);
                                continue;
                            }
                        } else {
                            // Assume pos is an aggregate type
                            x = pos.getDouble("x");
                            y = pos.getDouble("y");
                            z = pos.getDouble("z");
                        }

                        //ConsoleUtil.writeInfo("Probe " + i + " position: (" + x + ", " + y + ", " + z + ")");
                        validProbes.push({
                            id: i,
                            x: x.toFixed(2),
                            y: y.toFixed(2),
                            z: z.toFixed(2),
                            confidence: confidence.toFixed(2)
                        });
                    }
                }
                //ConsoleUtil.writeInfo("Valid probes: " + validProbes.length);
            } else {
                //ConsoleUtil.writeInfo("Error: Invalid data - confidences=" + confidences + ", positionsArray=" + positionsArray);
            }
        } else {
            ConsoleUtil.writeInfo("Error: PVs not connected");
        }
    } catch (e) {
        ConsoleUtil.writeInfo("Error reading probe data: " + e);
    }

    // Adjust rows
    var desiredRows = Math.max(1, validProbes.length);
    while (table.getRowCount() > desiredRows) {
        table.deleteRow(table.getRowCount() - 1);
    }
    while (table.getRowCount() < desiredRows) {
        var rowIndex = table.appendRow();
        table.setCellText(rowIndex, 0, rowIndex.toString());
        for (var col = 1; col < 5; col++) {
            table.setCellText(rowIndex, col, "-");
        }
    }

    // Update table
    try {
        if (table.getColumnCount() >= 5) {
            for (var i = 0; i < validProbes.length; i++) {
                table.setCellText(i, 0, validProbes[i].id.toString());
                table.setCellText(i, 1, validProbes[i].x);
                table.setCellText(i, 2, validProbes[i].y);
                table.setCellText(i, 3, validProbes[i].z);
                table.setCellText(i, 4, validProbes[i].confidence);
            }
            for (var i = validProbes.length; i < table.getRowCount(); i++) {
                table.setCellText(i, 0, i.toString());
                for (var col = 1; col < 5; col++) {
                    table.setCellText(i, col, "-");
                }
            }
        } else {
            ConsoleUtil.writeInfo("Cannot update table: insufficient columns (" + table.getColumnCount() + ")");
        }
    } catch (e) {
        ConsoleUtil.writeInfo("Error updating table: " + e);
    }

    table.refresh();
}

updateProbeTable(pvs, widget);