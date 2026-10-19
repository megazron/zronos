// zronOS playground: an in-memory robot folder fed to the zron WebAssembly binary.
(async function () {
  const $ = (id) => document.getElementById(id);
  const presets = await (await fetch("presets.json")).json();
  let files = {}, current = "", mode = "demo";

  // ---- presets + file tabs ----
  for (const p of presets) $("preset").add(new Option(p.title, p.id));
  function load(id) {
    const p = presets.find((x) => x.id === id);
    files = { ...p.files };
    current = p.manifest;
    tabs();
  }
  function tabs() {
    const box = $("files");
    box.textContent = "";
    const names = Object.keys(files).sort((a, b) => (a.endsWith(".manifest") ? -1 : b.endsWith(".manifest") ? 1 : a.localeCompare(b)));
    for (const n of names) {
      const b = document.createElement("button");
      b.textContent = n;
      b.setAttribute("role", "tab");
      b.setAttribute("aria-selected", String(n === current));
      b.onclick = () => { current = n; tabs(); };
      box.appendChild(b);
    }
    $("editor").value = files[current];
  }
  $("editor").addEventListener("input", () => (files[current] = $("editor").value));
  $("editor").addEventListener("keydown", (e) => {
    if (e.key === "Tab") { e.preventDefault(); document.execCommand("insertText", false, "  "); }
  });
  $("preset").onchange = () => load($("preset").value);
  $("reset").onclick = () => load($("preset").value);
  load(presets[0].id);

  // ---- modes ----
  document.querySelectorAll("#modes button").forEach((b) => (b.onclick = () => {
    mode = b.dataset.mode;
    document.querySelectorAll("#modes button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
    for (const m of ["demo", "shell", "doctor"]) $("opt-" + m).hidden = m !== mode;
  }));
  $("secs").oninput = () => ($("secs-v").textContent = $("secs").value + " s");

  // ---- output colouring ----
  function render(text) {
    const out = $("out");
    out.textContent = "";
    for (const line of text.replace(/\n+$/, "").split("\n")) {
      const row = document.createElement("span");
      const m = line.match(/^(\[\s*[\d.]+s\]) (\S+\s*)\|(.*)$/);
      const tone = /E-STOP|BLOCKED|FAILED|FAIL\b|ERROR|NOT READY|UNHEALTHY/.test(line) ? "bad"
        : /WARN|DEGRADED|restart|missed/.test(line) ? "warn"
        : /\[ OK \]|HEALTHY|READY TO BOOT|is RUNNING/.test(line) ? "ok" : "";
      if (m) {
        const t = document.createElement("span"); t.className = "t"; t.textContent = m[1] + " ";
        const s = document.createElement("span"); s.className = m[2].trim() === "init" || m[2].trim() === "kernel" ? "k" : "src"; s.textContent = m[2] + "|";
        const r = document.createElement("span"); r.className = tone; r.textContent = m[3];
        row.append(t, s, r);
      } else {
        row.className = line.startsWith("zron>") ? "prompt" : tone;
        row.textContent = line;
      }
      row.append("\n");
      out.appendChild(row);
    }
    out.scrollTop = 0;
  }

  // ---- run the binary: a fresh instance per run, so every boot is clean ----
  function status(msg, cls) { const s = $("status"); s.textContent = msg; s.className = "status " + (cls || ""); }
  async function zron(args, stdinText) {
    let out = "", i = 0;
    const input = stdinText || "";
    const M = await createZron({
      print: (t) => (out += t + "\n"),
      printErr: (t) => (out += t + "\n"),
      stdin: () => (i < input.length ? input.charCodeAt(i++) : null),
    });
    M.FS.mkdir("/robot");
    for (const [n, t] of Object.entries(files)) M.FS.writeFile("/robot/" + n, t);
    let rc = 0;
    try { rc = M.callMain(args); } catch (e) { rc = typeof e.status === "number" ? e.status : 1; if (typeof e.status !== "number") out += String(e) + "\n"; }
    return { rc, out };
  }
  async function run() {
    const manifest = Object.keys(files).find((n) => n.endsWith(".manifest"));
    $("run").disabled = true;
    status("running…");
    const t0 = performance.now();
    let r;
    if (mode === "demo") r = await zron(["demo", "/robot/" + manifest, "--seconds", $("secs").value]);
    else if (mode === "doctor") r = await zron(["doctor", "/robot/" + manifest]);
    else {
      let script = $("shell").value.trim() + "\n";
      if (!/^(quit|exit|halt|bye)\s*$/m.test(script)) script += "quit\n";
      r = await zron(["run", "/robot/" + manifest], script);
    }
    render(r.out);
    $("run").disabled = false;
    status(`exit ${r.rc} · ${Math.round(performance.now() - t0)} ms`, r.rc === 0 ? "ok" : "bad");
  }
  $("run").onclick = run;
  document.addEventListener("keydown", (e) => { if ((e.ctrlKey || e.metaKey) && e.key === "Enter") { e.preventDefault(); run(); } });

  const v = await zron(["--version"]);
  $("ver").textContent = v.out.trim().replace("zronOS ", "v") + " · playground";
  $("run").disabled = false;
  $("run").textContent = "▶ Boot";
  status("ready: zron loaded as WebAssembly", "ok");
  run();
})();
