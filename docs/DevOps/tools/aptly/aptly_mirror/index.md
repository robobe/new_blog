---
title: Aptly create mirror for selected packages
tags:
    - aptly
    - mirror
---
aptly allows to create mirrors of remote Debian repositories, it can mirror selected packages using filter

## install

```bash title="/etc/apt/sources.list.d/aptly.list"

echo "deb [signed-by=/etc/apt/keyrings/aptly.asc] http://repo.aptly.info/release jammy main" \
  | sudo tee /etc/apt/sources.list.d/aptly.list > /dev/null

```

```bash title="download key"
sudo wget -O /etc/apt/keyrings/aptly.asc https://www.aptly.info/pubkey.txt
```

```bash
sudo apt update
sudo apt install aptly
```


## Demo
Create mirror for `` packages use docker to check the install


- Create
- Update
- Snapshot
- Publish
- Server

### Create
[create](https://www.aptly.info/doc/aptly/mirror/create/)

```bash title="create mirror"
#aptly mirror create <name> <archive url> <distribution> 

aptly mirror create \
  -architectures="arm64" \
  -filter="tcpdump|iputils-ping" \
  -filter-with-deps \
  mirror-demo \
  http://ports.ubuntu.com/ubuntu-ports \
  jammy \
  main

```

```bash
gpg --no-default-keyring \
    --keyring trustedkeys.gpg \
    --keyserver keyserver.ubuntu.com \
    --recv-keys 871920D1991BC93C
```

!!! Note ignore upstream signature

    ```bash title="ignore-signatures"
    # not recommended use -ignore-signatures flag
    aptly mirror create -ignore-signatures
    ```

### update
```bash
# aptly mirror update <mirror name>
aptly mirror update mirror-demo
```

### Snapshot

```bash 
#aptly snapshot create <name> from mirror <mirror name>
aptly snapshot create mirror-demo-v1 from mirror mirror-demo
```

### Create key

```bash
gpg --full-generate-key
#
public and secret key created and signed.

pub   rsa4096 2025-12-09 [SC]
      4C4B6FD1CFF42336F74470225127A6636EB51D12
uid                      dev <dev@dev.com>
sub   rsa4096 2025-12-09 [E]
```

- Key type → RSA and RSA
- Key size → 4096 bits
- Expiration → 0 = no expiration
- passphrase → dev


```bash
gpg --list-secret-keys --keyid-format LONG
```

```bash title="export key to use by apt"
gpg --export -a "4C4B6FD1CFF42336F74470225127A6636EB51D12" > aptly.asc
```


```bash title="use the key"
deb [signed-by=aptly.asc] http://your-server/apt jammy main
```


### Publish

```bash
aptly publish snapshot -gpg-key="4C4B6FD1CFF42336F74470225127A6636EB51D12" \
mirror-demo-v1
```

---

## Demo
Update the mirror with another package

```bash
aptly mirror edit \
  -architectures="arm64" \
  -filter="tcpdump|iputils-ping | net-tools" \
  -filter-with-deps \
  mirror-demo 
```


### update
```bash
# aptly mirror update <mirror name>
aptly mirror update mirror-demo
```

### snapshot
```bash 
#aptly snapshot create <name> from mirror <mirror name>
aptly snapshot create mirror-demo-v2 from mirror mirror-demo
```

### publish (switch)
Command switches in-place published repository with new snapshot contents [more](https://www.aptly.info/doc/aptly/publish/switch/)


```bash title="publish list"
aptly publish list
#
Published repositories:
  * ./jammy (origin: Ubuntu) [arm64] publishes {main: [mirror-demo-v1]: Snapshot from mirror [mirror-demo]: http://ports.ubuntu.com/ubuntu-ports/ jammy}
```

```bash
aptly publish switch -gpg-key="4C4B6FD1CFF42336F74470225127A6636EB51D12" jammy mirror-demo-v2
#
Published snapshot repository ./jammy (origin: Ubuntu) [arm64] publishes {main: [mirror-demo-v2]: Snapshot from mirror [mirror-demo]: http://ports.ubuntu.com/ubuntu-ports/ jammy} has been successfully switched to new source.
```

```bash title="serve"
aptly serve
Serving published repositories, recommended apt sources list:

# ./jammy (origin: Ubuntu) [arm64] publishes {main: [mirror-demo-v2]: Snapshot from mirror [mirror-demo]: http://ports.ubuntu.com/ubuntu-ports/ jammy}
deb http://127.0.0.1:8080/ jammy main
```

### usage

Run minimal ubuntu 24.04 arm docker , edit the source list and try to install one of the package from the custom mirror


- Copy exported key
- Remove sources.list.d and update sources.list

#### files
- [aptly.asc](code/aptly.asc)
- [Dockerfile](code/Dockerfile)
- [sources.list](code/sources.list)

```dockerfile
FROM ubuntu:24.04

COPY aptly.asc /etc/apt/keyrings/aptly.asc
COPY sources.list /etc/apt/sources.list
RUN chmod 644 /etc/apt/keyrings/aptly.asc \
    && rm -rf /etc/apt/sources.list.d

CMD ["/bin/bash"]
```

```bash title="/etc/apt/sources.list"
deb [signed-by=/etc/apt/keyrings/aptly.asc] http://127.0.0.1:8080 jammy main

```

```
docker build --platform linux/arm64 -t ubuntu/24.04:arm64 .
```

```
docker run --rm -it --net host --hostname test --platform linux/arm64 ubuntu/24.04:arm64 
```

#### Test

```
apt update
apt install tcpdump
```


!!! Note use no key
    ```bash
    deb [trusted=yes] http://example.com/debian stable main
    ```

---

## use file as filter list

```bash
aptly mirror edit \
  -architectures="arm64" \
  -filter="@filter.txt" \
  -filter-with-deps \
  mirror-demo 
```

```title="filter.txt"
(
curl |
vim |
tcpdump |
libpcap* |
net-tools
)
```