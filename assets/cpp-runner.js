function resultParts(stdout, stderr, fallbackMessage) {
    const parts = [stdout, stderr].filter(Boolean);
    return parts.length > 0 ? parts : [fallbackMessage].filter(Boolean);
}

document.querySelectorAll(".cpp-runner").forEach((runner) => {
    const editor = runner.querySelector(".cpp-editor");
    const output = runner.querySelector(".cpp-output");
    const runButton = runner.querySelector(".cpp-run-button");

    if (!editor || !output || !runButton) {
        return;
    }

    runButton.addEventListener("click", async () => {
        output.hidden = false;
        output.textContent = "Compiling and running...";
        runButton.disabled = true;

        const requestBody = {
            compiler: "gcc-head",
            code: editor.value,
            options: "warning,gnu++23",
            stdin: "",
            "compiler-option-raw": "",
            "runtime-option-raw": ""
        };

        try {
            const response = await fetch(
                "https://wandbox.org/api/compile.json",
                {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json"
                    },
                    body: JSON.stringify(requestBody)
                }
            );

            if (!response.ok) {
                throw new Error(
                    `Wandbox request failed: ${response.status} ${response.statusText}`
                );
            }

            const result = await response.json();

            const messages = [
                ...resultParts(
                    result.compiler_output,
                    result.compiler_error,
                    result.compiler_message
                ),
                ...resultParts(
                    result.program_output,
                    result.program_error,
                    result.program_message
                ),
                result.signal
            ].filter(Boolean);

            output.textContent =
                messages.length > 0
                    ? messages.join("\n")
                    : "Program finished without output.";
        } catch (error) {
            console.error(error);
            output.textContent = `Error: ${error.message}`;
        } finally {
            runButton.disabled = false;
        }
    });
});
