function computeProbeCoordinates(input) {
    var probePositions = input.getParameter("Probe_Positions");
    var confidences = input.getParameter("Probe_Confidences");
    var xCoords = [];
    var yCoords = [];
    for (var i = 0; i < 3; i++) {
        // Only include probes with non-zero confidence
        if (confidences[i] > 0.0) {
            xCoords.push(probePositions[i].x);
            yCoords.push(probePositions[i].y);
        }
    }
    return {
        "Probe_X_Coordinates": xCoords,
        "Probe_Y_Coordinates": yCoords
    };
}