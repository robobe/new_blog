from mkdocs.plugins import BasePlugin
from mkdocs.config import config_options
import re
import requests

class MyPlugin(BasePlugin):
    config_scheme = (
        ('message', config_options.Type(str, default='Hello, MkDocs!')),
    )

    def on_pre_build(self, config):
        print(f"MyPlugin: {self.config['message']}")

    def on_page_markdown(self, markdown, page, config, files):
        """Find and replace `{{PLUGIN uri="..."}}` with the fetched content."""
        
        pattern = r'{{PLUGIN\s+uri="([^"]+)"}}'

        def replace_with_fetched_content(match):
            uri = match.group(1)
            try:
                response = requests.get(uri, timeout=5)
                if response.status_code == 200:
                    return f"{response.text}"
                else:
                    return f"**Error fetching URI ({response.status_code})**"
            except requests.exceptions.RequestException as e:
                return f"**Error fetching URI: {e}**"

        return re.sub(pattern, replace_with_fetched_content, markdown)