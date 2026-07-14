(function () {
  const panZoomInstances = new WeakMap();

  function initializeMermaid() {
    if (typeof mermaid === "undefined") {
      console.error("Mermaid was not loaded");
      return;
    }

    mermaid.initialize({
      startOnLoad: false,
      securityLevel: "strict",
      theme: document.body.getAttribute("data-md-color-scheme") === "slate"
        ? "dark"
        : "default"
    });

    renderDiagrams();
  }

  async function renderDiagrams() {
    normalizeMermaidFences();

    const diagrams = Array.from(
      document.querySelectorAll(".mermaid")
    ).filter((diagram) => !diagram.closest(".mermaid-viewer"));

    if (diagrams.length === 0) {
      return;
    }

    const unprocessedDiagrams = diagrams.filter(
      (diagram) => !diagram.hasAttribute("data-processed")
    );

    /*
     * mermaid.run() is preferred for manually controlled rendering
     * in current Mermaid versions.
     */
    if (unprocessedDiagrams.length > 0) {
      await mermaid.run({
        nodes: unprocessedDiagrams
      });
    }

    diagrams.forEach(createViewer);
  }

  function normalizeMermaidFences() {
    document.querySelectorAll("pre.mermaid > code").forEach((code) => {
      const pre = code.parentElement;
      const diagram = document.createElement("div");

      diagram.className = "mermaid";
      diagram.textContent = code.textContent;
      pre.replaceWith(diagram);
    });
  }

  function createViewer(diagram) {
    if (diagram.closest(".mermaid-viewer")) {
      return;
    }

    const svg = diagram.querySelector("svg");

    if (!svg) {
      return;
    }

    const viewer = document.createElement("div");
    viewer.className = "mermaid-viewer";
    applyLocalHeight(viewer, diagram);

    const toolbar = document.createElement("div");
    toolbar.className = "mermaid-toolbar";

    const zoomInButton = createButton("+", "Zoom in");
    const zoomOutButton = createButton("−", "Zoom out");
    const resetButton = createButton("Reset", "Reset diagram");
    const fullscreenButton = createButton("⛶", "Open full screen");

    toolbar.append(
      zoomInButton,
      zoomOutButton,
      resetButton,
      fullscreenButton
    );

    const canvas = document.createElement("div");
    canvas.className = "mermaid-canvas";

    diagram.parentNode.insertBefore(viewer, diagram);
    viewer.append(toolbar, canvas);
    canvas.appendChild(diagram);

    /*
     * Mermaid sometimes places explicit width and height values on its SVG.
     * Removing them allows the viewer container to control the visible area.
     */
    svg.removeAttribute("width");
    svg.removeAttribute("height");
    svg.style.width = "100%";
    svg.style.height = "100%";

    const instance = svgPanZoom(svg, {
      zoomEnabled: true,
      controlIconsEnabled: false,
      fit: true,
      center: true,
      minZoom: 0.2,
      maxZoom: 20,
      zoomScaleSensitivity: 0.25,
      dblClickZoomEnabled: true,
      mouseWheelZoomEnabled: true,
      preventMouseEventsDefault: true
    });

    panZoomInstances.set(viewer, instance);

    zoomInButton.addEventListener("click", () => {
      instance.zoomIn();
    });

    zoomOutButton.addEventListener("click", () => {
      instance.zoomOut();
    });

    resetButton.addEventListener("click", () => {
      instance.resetZoom();
      instance.center();
      instance.fit();
    });

    fullscreenButton.addEventListener("click", async () => {
      try {
        if (!document.fullscreenElement) {
          await viewer.requestFullscreen();
        } else {
          await document.exitFullscreen();
        }
      } catch (error) {
        console.error("Could not enter full screen:", error);
      }
    });

    document.addEventListener("fullscreenchange", () => {
      setTimeout(() => {
        instance.resize();
        instance.fit();
        instance.center();
      }, 100);
    });

    const resizeObserver = new ResizeObserver(() => {
      instance.resize();
    });

    resizeObserver.observe(canvas);
  }

  function createButton(text, label) {
    const button = document.createElement("button");

    button.type = "button";
    button.className = "mermaid-toolbar-button";
    button.textContent = text;
    button.setAttribute("aria-label", label);
    button.title = label;

    return button;
  }

  function applyLocalHeight(viewer, diagram) {
    const previousElement = diagram.previousElementSibling;
    const localHeight =
      diagram.getAttribute("data-height") ||
      diagram.parentElement?.getAttribute("data-mermaid-height") ||
      previousElement?.getAttribute("data-mermaid-height") ||
      previousElement?.getAttribute("data-height");

    if (localHeight) {
      viewer.style.setProperty("--mermaid-canvas-height", localHeight);
    }
  }

  /*
   * Normal browser load.
   */
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initializeMermaid);
  } else {
    initializeMermaid();
  }

  /*
   * MkDocs Material instant navigation support.
   *
   * Material replaces page content without necessarily reloading the browser,
   * so diagrams must be initialized after navigation as well.
   */
  if (typeof document$ !== "undefined") {
    document$.subscribe(() => {
      initializeMermaid();
    });
  }
})();
