// Keep last N points
const MAX_POINTS = 600; // ~120s at 5 Hz

function makeTraces(prefix, n, yLabel) {
  const traces = [];
  for (let i = 0; i < n; i++) {
    traces.push({ x: [], y: [], mode: "lines", name: `${prefix}${i}` });
  }
  return traces;
}

const quatTraces = [
  {x:[], y:[], mode:"lines", name:"q0"},
  {x:[], y:[], mode:"lines", name:"q1"},
  {x:[], y:[], mode:"lines", name:"q2"},
  {x:[], y:[], mode:"lines", name:"q3"},
];

const eulerTraces = [
  {x:[], y:[], mode:"lines", name:"roll"},
  {x:[], y:[], mode:"lines", name:"pitch"},
  {x:[], y:[], mode:"lines", name:"yaw"},
];

const omegaTraces = [
  {x:[], y:[], mode:"lines", name:"wx"},
  {x:[], y:[], mode:"lines", name:"wy"},
  {x:[], y:[], mode:"lines", name:"wz"},
];

const rpmTraces = makeTraces("RW", 3);
const torqueTraces = [
  {x:[], y:[], mode:"lines", name:"RW0 Cmd Torque"},
  {x:[], y:[], mode:"lines", name:"RW1 Cmd Torque"},
  {x:[], y:[], mode:"lines", name:"RW2 Cmd Torque"},
];

Plotly.newPlot("quatPlot", quatTraces, {
  title: "Quaternion (estimated)",
  xaxis: { title: "Time (s)" },
  yaxis: { title: "q" },
  margin: { t: 40, l: 50, r: 20, b: 40 }
});

Plotly.newPlot("eulerPlot", eulerTraces, {
  title: "Euler Angles (estimated)",
  xaxis: { title: "Time (s)" },
  yaxis: { title: "eul" },
  margin: { t: 40, l: 50, r: 20, b: 40 }
});

Plotly.newPlot("omegaPlot", omegaTraces, {
  title: "Body Rates (estimated)",
  xaxis: { title: "Time (s)" },
  yaxis: { title: "rad/s" },
  margin: { t: 40, l: 50, r: 20, b: 40 }
});

Plotly.newPlot("rpmPlot", rpmTraces, {
  title: "Wheel RPM",
  xaxis: { title: "Time (s)" },
  yaxis: { title: "RPM" },
  margin: { t: 40, l: 50, r: 20, b: 40 }
});

Plotly.newPlot("torquePlot", torqueTraces, {
  title: "Commanded Wheel Torque",
  xaxis: { title: "Time (s)" },
  yaxis: { title: "N·m" },
  margin: { t: 40, l: 50, r: 20, b: 40 }
});

function pushPoint(trace, t, y) {
  trace.x.push(t);
  trace.y.push(y);
  if (trace.x.length > MAX_POINTS) {
    trace.x.shift();
    trace.y.shift();
  }
}

async function poll() {
  try {
    const resp = await fetch("/telemetry", { cache: "no-store" });
    if (!resp.ok) return;
    const data = await resp.json();

    const t = (data.t_us || 0) * 1e-6;

    // Quaternion
    for (let i = 0; i < 4; i++) pushPoint(quatTraces[i], t, data.q[i]);

    // Euler Angles
    if (Array.isArray(data.eul) && data.eul.length === 3) {
	  for (let i = 0; i < 3; i++) {
	    pushPoint(eulerTraces[i], t, data.eul[i]);
	  }
	}

    // Omega
    for (let i = 0; i < 3; i++) pushPoint(omegaTraces[i], t, data.omega[i]);

    // RPM
    for (let i = 0; i < 3; i++) pushPoint(rpmTraces[i], t, data.wheel_rpm[i]);

    // Commanded torque
    for (let i = 0; i < 3; i++) pushPoint(torqueTraces[i], t, data.wheel_torque_cmd_nm[i]);

    Plotly.redraw("quatPlot");
    Plotly.redraw("eulerPlot");
    Plotly.redraw("omegaPlot");
    Plotly.redraw("rpmPlot");
    Plotly.redraw("torquePlot");
  } catch (e) {
    // ignore transient errors
  }
}

setInterval(poll, 200); // 5 Hz
poll();
