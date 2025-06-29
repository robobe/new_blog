import os
import pathlib

def define_env(env):
    @env.macro
    def page_folder_links():
        # Get the URL and path of the current page
        url = env.page.url.rstrip('/')  # e.g., blog/2024/intro
        mk_path = env.page.file.src_path   # e.g., blog/2024/intro.md
        path = pathlib.Path(mk_path).parent.parent.as_posix()
        # Get the folder components (exclude the filename)
        folder_parts = path.split(os.sep)

        # Build partial URLs
        parts = []
        current_url = ""
        for part in folder_parts:
            current_url += "/" + part
            display_part = part.replace("_", " ")
            parts.append(f"[{display_part}]({current_url}/)")

        return " / ".join(parts)
