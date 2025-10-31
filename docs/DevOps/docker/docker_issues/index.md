---
title: Docker issues
tags:
    - docker
    - issue
---


{{ page_folder_links() }}

## Network
failed to start daemon: Error initializing network controller: error obtaining controller instance: failed to register "bridge" driver: failed to add

### Resolve
TODO: understand more
```
{
    "bridge": "none",
    "iptables": false,
    "default-address-pools": [
        {"base": "172.30.0.0/16", "size": 24}
  ]
}

```

```bash title="downgrade docker"
sudo apt-get install -y docker-ce=5:27.5* docker-ce-cli=5:27.5* --allow-downgrades
```