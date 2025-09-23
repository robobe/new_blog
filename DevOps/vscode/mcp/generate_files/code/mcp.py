import logging
from pathlib import Path
from mcp.server.fastmcp import FastMCP

# --- Setup ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("template-mcp")

mcp = FastMCP("TemplateBuilder")
log.info("MCP server initialized. -----------")
TEMPLATES_DIR = Path("templates")

# -------------------------------------------------
# TOOL → Generate file from a template
# -------------------------------------------------
@mcp.tool()
def generate_file(template: str, output_name: str, **kwargs) -> dict:
    """
    Generate a new file from a given template.

    Args:
        template: filename in templates/ (e.g. gazebo.launch.py)
        output_name: new file name (e.g. my_gazebo.launch.py)
        kwargs: placeholder values to substitute
    """
    template_path = TEMPLATES_DIR / template
    if not template_path.exists():
        raise ValueError(f"Template '{template}' not found in {TEMPLATES_DIR}")

    content = template_path.read_text()

    # Replace placeholders like {{robot_name}} with values from kwargs
    for key, value in kwargs.items():
        content = content.replace(f"{{{{{key}}}}}", str(value))

    log.info(f"Generated {output_name} from {template}")
    return {"filename": output_name, "content": content}

# -------------------------------------------------
# PROMPT → Choose the right template
# -------------------------------------------------
@mcp.prompt()
def choose_template(subject: str) -> str:
    """
    Guide the AI to pick the right template based on the subject.
    """
    available = [f.name for f in TEMPLATES_DIR.glob("*.launch.py")]
    prompt = f"""
    The user wants: "{subject}"

    Available templates:
    {available}

    Select the best template file name and call generate_file with:
      - template: <chosen template>
      - output_name: a descriptive filename for the project
      - kwargs: any substitutions needed (e.g. world="my_world.sdf")
    """
    log.info(f"Prompting AI with subject: {subject}")
    log.info(available)
    return prompt

# -------------------------------------------------
# Entrypoint
# -------------------------------------------------
if __name__ == "__main__":
    log.info("Starting TemplateBuilder MCP server...")
    mcp.run()
