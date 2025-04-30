---
tags:
    - aptly
    - rest
---

# Aptly REST Api
Aptly operations are also available via REST API served with `aptly api serve`.
The example and API reference refer to version 1.6.1

!!! note "version 1.6.1"
    list and reference [Aptly REST API](https://www.aptly.info/doc/api/swagger/)
     

## Run and Test
```bash title="run aptly REST api server"
aptly api serve
```

### Test REST Api
```bash title="get server version"
curl http://localhost:8080/api/version | jq .

#
{
    "Version":"1.6.1"
}

```



     

## Files
Upload **debs file** to aptly location  
from this location aptly update it's repositories
After adding to a repository, the files are removed by default.

```bash title="upload file"
# POST /api/files/{dir}
# curl -X POST -F file=@{deb file} http://localhost:8080/api/files/uploads
curl -X POST -F file=@my-tool_0.0.1_amd64.deb http://localhost:8080/api/files/uploads
```

```bash title="list files in directory"
#GET /api/files/{dir}
curl http://localhost:8080/api/files/uploads
```


## Repository
A local repository is a collection of versionned packages (usually custom packages created internally).

Packages can be added, removed, moved or copied between repos.

Local repositories can be published (either directly or via snapshot) to be used a APT source on a debian based system.

```bash title="list repos"
# GET /api/repos
curl http://localhost:8080/api/repos | jq .
```


```bash title="create repo"
POST /api/repos

curl -X POST http://localhost:8080/api/repos \
-H 'Content-Type: application/json' \
--data '{
    "Name": "my-repo"
    "Comment":"my application repo",
    "DefaultDistribution":"jammy",
    "DefaultComponent":"main"
    }' 
```

```bash title="add uploaded folder"
# POST /api/repos/{name}/file/{dir}
curl -X POST http://localhost:8080/api/repos/my-repo/file/uploads
```

## Publish
Publish snapshot or local repo as Debian repository to be used as APT source on Debian based systems.

```bash title="list published repositories"
# GET /api/publish
curl http://localhost:8080/api/publish | jq .
```

### create published repository
Publish a local repository or snapshot

The prefix may contain a storage specifier, e.g. s3:packages/, or it may also be empty to publish to the root directory.

```bash title="create"
# /api/publish/{prefix}
curl -X POST http://localhost:8080/api/publish/ \
-H 'Content-Type: application/json' \
--data '
{
    "Distribution": "jammy",
    "Sources": [{"Name": "my-repo"}],
    "SourceKind": "local",
    "Signing": {
        "Skip": true
    }
}' 
```

```bash title="update"
# PUT /api/publish/{prefix}/{distribution}
# Empty prefix

curl -X PUT http://localhost:8080/api/publish//jammy
-H 'Content-Type: application/json' \
--data '
{
    "ForceOverwrite": true,
    "Architectures": [
        "amd64"
    ],
    "SourceKind": "local",
    "Sources": [
    {
      "Component": "main",
      "Name": "my-repo"
    },
    "Signing": {
        "Skip": true
  },
}'
```

```bash title="show published repository"
curl http://localhost:8080/api/publish/repos/jammy | jq .

```


---

## Demo: upload deb package to exists repository

The demo base and how to create [simple shell debian package]()

The demo assume that the repository exists and we all ready publish it. no snapshot 

Demo steps:

- Upload deb file
- Add file to repository
- Update publisher
- Serve
- Update local apt

### Pre

```bash title="run aptly"
aptly serve
```

```bash title="create repository using aptly cli"
REPO_NAME="my-repo"

aptly -distribution="jammy" -architectures="amd64" -comment="my repo desc" \
repo \
create \
${REPO_NAME}
```

```bash title="publish repo using aptly cli"
REPO_NAME="my-repo"

aptly -distribution="jammy" -architectures="amd64" -skip-signing="true" \
publish \
repo \
${REPO_NAME}

##
#Now you can add following line to apt sources:
#  deb http://your-server/ jammy main
```

### rest api
```bash
aptly api serve
```

### Upload

```bash
FILE=my-tool_0.0.1_amd64.deb
curl -X POST -F file=@${FILE} http://localhost:8080/api/files/uploads
```

### Update repository



```bash
REPO_NAME="my-repo"
curl -X POST http://localhost:8080/api/repos/${REPO_NAME}/file/uploads

```

### Update publisher

```bash
REPO_NAME="my-repo"

curl -X PUT http://localhost:8080/api/publish//jammy \
-H 'Content-Type: application/json' \
--data '
{
    "ForceOverwrite": true,
    "Architectures": [
        "amd64"
    ],
    "SourceKind": "local",
    "Sources": [
        {
        "Component": "main",
        "Name": "${REPO_NAME}"
        }
    ],
    "Signing": {
        "Skip": true
  }
}'
```


### Serve
- Serve
- Add to `sources.list` or `sources.list.d`
- Run `apt update`
- Install package

```bash
# serve with aptly or use nginx
aptly serve
# ./jammy [amd64] publishes {main: [my-repo]: my repo desc}
deb http://127.0.0.1:8080/ jammy main
```

!!! tip "repositories URL"
    When run `aptly server` it print all the publish repositories and the deb line add to the sources.list

    ```bash
    # repos/jammy [amd64] publishes {main: [my-repo]}
    deb http://lap:8080/repos/ jammy main
    ```
     
```bash
sudo vim /etc/apt/sources.list.d/my_repo.list
```

```
deb [trusted=yes] http://127.0.0.1:8080/ jammy main
```

```
sudo apt update
```

```bash
apt search my-tool
#
Sorting... Done
Full Text Search... Done
my-tool/jammy 0.0.1 amd64
  My Tool – brief description
```

## VSCode

Using vscode task to automate the above rest api calls

- Upload deb file
- Add files to repository
- Update publisher

!!! note "aptly api"
    Run server

    ```bash
    aptly api serve
    ```
     

### Upload task

!!! note "VSCode command variable"
     Using [command variable](https://marketplace.visualstudio.com/items?itemName=rioj7.command-variable) extension to pick deb file for upload


```json title="task section: upload file"
{
"label": "upload deb to aptly",
"type": "shell",
"command": "curl",
"args": [
    "-X",
    "POST",
    "-F",
    "file=@${input:file_path}",
    "http://localhost:8080/api/files/uploads"
],
"problemMatcher": []
}
```

```json title="input add pickFile"
{
    "id": "file_path",
    "type": "command",
    "command": "extension.commandvariable.file.pickFile",
    "args":{
        "include": "**/*.deb"
    }
}
```

### Update repo

```json title="task: update repository"
{
    "label": "add/update files to repository",
    "type": "shell",
    "command": "curl",
    "args": [
        "-X",
        "POST",
        "http://localhost:8080/api/repos/${input:repo_name}/file/uploads"
    ],
    "problemMatcher": []
}
```

```json title="input: repo name"
{
    "id": "repo_name",
    "default": "my-repo",
    "type": "promptString",
    "description": "my repository name"
}
```

### Update publish

!!! note "no prefix"
    The double `//` is because no prefix define
     

```bash title="task: update publish finish json part"
{
    "label": "update publish",
    "type": "shell",
    "command": "curl",
    "args": [
        "-X",
        "PUT",
        "http://localhost:8080/api/publish//jammy",
        "-H",
        "Content-Type: application/json",
        "--data",
        "{\"ForceOverwrite\": true, \"Architectures\": [\"amd64\"],\"SourceKind\": \"local\",\"Sources\": [{\"Component\": \"main\",\"Name\": \"${REPO_NAME}\"}],\"Signing\": {\"Skip\": true}}"

    ],
    "problemMatcher": []
}
```

[download vscode full tasks.json file](docs/DevOps/tools/aptly/aptly_rest_api/code/tasks.json){:target="_blank"}

### Check

- update sources.list
- run aptly


```bash
apt update
```

```bash
apt search my-tool
#
Sorting... Done
Full Text Search... Done
my-tool/jammy 0.0.2 amd64
  My Tool – brief description
```

```bash
apt list -a my-tool
#
Listing... Done
my-tool/jammy 0.0.2 amd64
my-tool/jammy 0.0.1 amd64
```